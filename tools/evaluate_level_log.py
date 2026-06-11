# =============================================================================
#  evaluate_level_log.py — offline evaluation & HTML report for the
#  M5Stack CoreS3 Static Level Logger CSV files
# -----------------------------------------------------------------------------
#  Usage
#    python evaluate_level_log.py LOG.csv [MORE.csv ...] [options]
#
#    Default flow: a time-series window opens; DRAG horizontally to select one
#    or more ranges of measurement points (keys: u = undo last, r = reset,
#    a = select all, ENTER / close window = done). Selections are saved next
#    to the log as <log>.selections.json so a report can be regenerated
#    reproducibly with --reuse (no GUI).
#
#    Options
#      -o FILE        output report path (default: <first log>_report.html)
#      --all          skip the GUI, evaluate the whole log as one selection
#      --reuse        skip the GUI, reuse the saved .selections.json
#      --title TEXT   report title
#      --settled-only use only settled rows for statistics (default: on)
#      --include-moving  also include settled=0 rows in the statistics
#
#  Dependencies: numpy, pandas, matplotlib — AUTO-INSTALLED on first run if
#  missing (via `<this python> -m pip install`). Manual one-liner:
#      python -m pip install numpy pandas matplotlib
#  Runners that understand PEP 723 inline metadata (e.g. `uv run`) handle the
#  dependencies in an isolated environment automatically.
# =============================================================================
# /// script
# requires-python = ">=3.9"
# dependencies = ["numpy", "pandas", "matplotlib"]
# ///

import argparse
import base64
import importlib.util
import io
import json
import math
import os
import subprocess
import sys
import datetime as _dt


def _ensure_deps():
    """Install missing third-party packages with the interpreter's own pip.
    Uses --user outside virtualenvs; on any failure, prints the manual
    command instead of leaving a half-configured environment."""
    missing = [m for m in ("numpy", "pandas", "matplotlib")
               if importlib.util.find_spec(m) is None]
    if not missing:
        return
    print(f"[setup] missing packages: {', '.join(missing)} — installing ...")
    cmd = [sys.executable, "-m", "pip", "install"]
    in_venv = sys.prefix != getattr(sys, "base_prefix", sys.prefix)
    if not in_venv:
        cmd.append("--user")
    try:
        subprocess.check_call(cmd + missing)
        importlib.invalidate_caches()
    except Exception:
        raise SystemExit(
            "[setup] automatic install failed. Install manually with:\n"
            f"    {sys.executable} -m pip install numpy pandas matplotlib\n"
            "(or create a venv first: python -m venv .venv && activate it)")
    still = [m for m in missing if importlib.util.find_spec(m) is None]
    if still:
        raise SystemExit(
            f"[setup] {', '.join(still)} installed but not importable — restart:\n"
            f"    {sys.executable} {' '.join(sys.argv)}")
    print("[setup] dependencies ready.")


_ensure_deps()

import numpy as np
import pandas as pd
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse, Circle

# -----------------------------------------------------------------------------
# Configuration (mirrors the config.h philosophy: everything tunable up here)
# -----------------------------------------------------------------------------
# Mounting: M5Stack on a 145 x 100 x 1 mm steel plate.
# The 145 mm side lies along the screen's SHORT edge -> tilted by PITCH.
# The 100 mm side lies along the screen's LONG  edge -> tilted by ROLL.
PLATE_ALONG_PITCH_MM = 145.0
PLATE_ALONG_ROLL_MM  = 100.0
PLATE_NOTE = ("M5Stack CoreS3 on a 145 x 100 x 1 mm steel plate; "
              "145 mm along pitch, 100 mm along roll. NOTE: a 1 mm plate can "
              "bow under clamping by more than the figures below — the IMU "
              "cannot see plate bending.")

TARGET_DEG    = 0.10     # accuracy target circle drawn on the 2-D plot
MIN_DWELL_S   = 3.0      # contiguous settled runs shorter than this are ignored
ALLAN_MIN_PTS = 60       # minimum points in a contiguous settled run for Allan

# NOVA (Hexagon / Leica Geosystems) palette — light mode, matches the dashboard
NOVA = {
    "primary":  "#005198",  # pitch
    "info":     "#33B4F2",  # roll
    "success":  "#6DBD55",
    "warning":  "#F19724",  # temperature
    "error":    "#BC1C1C",
    "on":       "#121623",
    "onvar":    "#646E78",
    "surface":  "#F8FAFD",
    "card":     "#FFFFFF",
    "outline":  "#858C99",
    "outvar":   "#E6EAF0",
    "brand":    "#83C410",
}
SELECTION_COLORS = ["#005198", "#28721E", "#A144EA", "#8E5515", "#0E6991",
                    "#BC1C1C", "#646E78", "#F569E2"]

DEG_PER_MG = 0.0573      # small-angle: 1 mg of accel offset ~ 0.0573 deg


# -----------------------------------------------------------------------------
# Loading
# -----------------------------------------------------------------------------
def load_logs(paths):
    """Load and concatenate logger CSVs. Returns a DataFrame with a continuous
    time axis t_s (seconds since the start of the first file; later files are
    appended after a visible gap) and wall-clock timestamps where available."""
    frames, t_offset, sessions = [], 0.0, []
    for i, p in enumerate(paths):
        df = pd.read_csv(p, on_bad_lines="skip")        # tolerate a torn last line
        need = ["millis", "pitch_deg", "roll_deg", "settled", "calibrated",
                "ax_g", "ay_g", "ax_raw_g", "ay_raw_g",
                "imu_temp_c", "event_flag", "event_label"]
        missing = [c for c in need if c not in df.columns]
        if missing:
            raise SystemExit(f"{p}: missing columns {missing} — is this a level_log CSV?")
        for c in need[:-1]:
            df[c] = pd.to_numeric(df[c], errors="coerce")
        df = df.dropna(subset=["millis", "pitch_deg", "roll_deg"]).reset_index(drop=True)
        df["wall"] = pd.to_datetime(df.get("timestamp_iso"), errors="coerce")
        df["t_s"] = (df["millis"] - df["millis"].iloc[0]) / 1000.0 + t_offset
        df["session"] = i
        df["source"] = os.path.basename(p)
        t_offset = df["t_s"].iloc[-1] + 30.0            # visible inter-file gap
        sessions.append((os.path.basename(p), len(df)))
        frames.append(df)
    data = pd.concat(frames, ignore_index=True)
    data["event_label"] = data["event_label"].fillna("")
    return data, sessions


def recovered_offsets(df):
    """The firmware logs raw and corrected accel; the applied calibration
    offset is recoverable as raw - corrected (per axis, in g)."""
    ox = (df["ax_raw_g"] - df["ax_g"]).round(6)
    oy = (df["ay_raw_g"] - df["ay_g"]).round(6)
    return ox, oy


def segment_dwells(df):
    """Contiguous settled==1 runs, ignoring runs shorter than MIN_DWELL_S.
    Returns a DataFrame of dwell statistics."""
    s = (df["settled"] == 1).astype(int)
    grp = (s.diff().fillna(0) != 0).cumsum()
    rows = []
    for _, g in df.groupby(grp):
        if g["settled"].iloc[0] != 1:
            continue
        dur = g["t_s"].iloc[-1] - g["t_s"].iloc[0]
        if dur < MIN_DWELL_S or len(g) < 3:
            continue
        rows.append({
            "t0": g["t_s"].iloc[0], "t1": g["t_s"].iloc[-1], "dur_s": dur,
            "n": len(g),
            "pitch_mean": g["pitch_deg"].mean(), "pitch_std": g["pitch_deg"].std(ddof=1),
            "roll_mean":  g["roll_deg"].mean(),  "roll_std":  g["roll_deg"].std(ddof=1),
            "temp_mean":  g["imu_temp_c"].mean(),
            "i0": g.index[0], "i1": g.index[-1],
        })
    d = pd.DataFrame(rows)
    if not d.empty:
        d.insert(0, "dwell", np.arange(1, len(d) + 1))
    return d


# -----------------------------------------------------------------------------
# Interactive range selection (drag on the time series)
# -----------------------------------------------------------------------------
def select_ranges_interactive(df, dwells):
    from matplotlib.widgets import SpanSelector

    fig, (axp, axr) = plt.subplots(2, 1, sharex=True, figsize=(12, 6))
    fig.canvas.manager.set_window_title("Drag to select ranges — u: undo, r: reset, a: all, ENTER: done")
    for ax, col, color, name in ((axp, "pitch_deg", NOVA["primary"], "pitch"),
                                 (axr, "roll_deg", NOVA["info"], "roll")):
        ax.plot(*gap_broken(df["t_s"], df[col]), color=color, lw=0.8)
        ax.set_ylabel(f"{name} [deg]")
        ax.grid(color=NOVA["outvar"], lw=0.5)
        for _, dw in dwells.iterrows():
            ax.axvspan(dw["t0"], dw["t1"], color=NOVA["success"], alpha=0.08)
    ev = df[df["event_flag"] == 1]
    for _, e in ev.iterrows():
        axp.axvline(e["t_s"], color=NOVA["brand"], lw=1)
        axp.annotate(str(e["event_label"]), (e["t_s"], axp.get_ylim()[1]),
                     fontsize=7, rotation=90, va="top", color=NOVA["brand"])
    axr.set_xlabel("time since log start [s]")
    fig.suptitle("DRAG on either plot to add a selection  |  u: undo   r: reset   a: whole log   ENTER/close: done",
                 fontsize=10)

    ranges, patches = [], []

    def add_range(a, b):
        if b - a < 0.5:
            return
        ranges.append((float(a), float(b)))
        col = SELECTION_COLORS[(len(ranges) - 1) % len(SELECTION_COLORS)]
        pr = [ax.axvspan(a, b, color=col, alpha=0.20) for ax in (axp, axr)]
        patches.append(pr)
        print(f"  selection S{len(ranges)}: {a:.1f} .. {b:.1f} s")
        fig.canvas.draw_idle()

    def on_key(event):
        if event.key == "u" and ranges:
            ranges.pop()
            for p in patches.pop():
                p.remove()
            print("  undo")
            fig.canvas.draw_idle()
        elif event.key == "r":
            while ranges:
                ranges.pop()
                for p in patches.pop():
                    p.remove()
            print("  reset")
            fig.canvas.draw_idle()
        elif event.key == "a":
            add_range(df["t_s"].iloc[0], df["t_s"].iloc[-1])
        elif event.key == "enter":
            plt.close(fig)

    spans = [SpanSelector(ax, add_range, "horizontal", useblit=True,
                          props=dict(alpha=0.15, facecolor=NOVA["primary"]))
             for ax in (axp, axr)]
    fig.canvas.mpl_connect("key_press_event", on_key)
    print("Selection window open — drag to select ranges, ENTER when done.")
    plt.show()
    del spans
    return ranges


def selections_sidecar_path(first_log):
    return os.path.splitext(first_log)[0] + ".selections.json"


# -----------------------------------------------------------------------------
# Statistics
# -----------------------------------------------------------------------------
def disp_um(angle_deg, length_mm):
    """Height displacement across a plate length for a tilt angle, in microns."""
    return math.tan(math.radians(angle_deg)) * length_mm * 1000.0


def eval_selection(df, dwells, t0, t1, settled_only=True):
    sel = df[(df["t_s"] >= t0) & (df["t_s"] <= t1)]
    used = sel[sel["settled"] == 1] if settled_only else sel
    out = {"t0": t0, "t1": t1, "n_total": len(sel), "n_used": len(used),
           "settled_only": settled_only}
    if len(used) < 3:
        out["valid"] = False
        return out
    out["valid"] = True
    for ax_name, col, plate in (("pitch", "pitch_deg", PLATE_ALONG_PITCH_MM),
                                ("roll", "roll_deg", PLATE_ALONG_ROLL_MM)):
        v = used[col].to_numpy()
        mean, std = float(np.mean(v)), float(np.std(v, ddof=1))
        pp = float(np.max(v) - np.min(v))
        # drift: linear fit over the selection, deg/min
        tt = used["t_s"].to_numpy()
        drift = float(np.polyfit(tt, v, 1)[0] * 60.0) if tt[-1] > tt[0] else 0.0
        out[ax_name] = {
            "mean": mean, "std": std, "pp": pp,
            "min": float(np.min(v)), "max": float(np.max(v)),
            "drift_deg_min": drift,
            "mean_um": disp_um(mean, plate),
            "std_um": disp_um(std, plate),
            "pp_um": disp_um(pp, plate),
            "plate_mm": plate,
        }
    # between-dwell repeatability inside the selection
    dsel = dwells[(dwells["t0"] >= t0) & (dwells["t1"] <= t1)] if not dwells.empty else dwells
    out["n_dwells"] = len(dsel)
    if len(dsel) >= 2:
        out["repeat_pitch_std"] = float(dsel["pitch_mean"].std(ddof=1))
        out["repeat_roll_std"] = float(dsel["roll_mean"].std(ddof=1))
    out["temp_mean"] = float(used["imu_temp_c"].mean())
    out["temp_span"] = float(used["imu_temp_c"].max() - used["imu_temp_c"].min())
    ox, oy = recovered_offsets(used)
    out["off_x_mg"] = float(ox.mean() * 1000.0)
    out["off_y_mg"] = float(oy.mean() * 1000.0)
    out["events"] = [(float(r["t_s"]), str(r["event_label"]))
                     for _, r in sel[sel["event_flag"] == 1].iterrows()]
    out["dwell_table"] = dsel
    return out


# -----------------------------------------------------------------------------
# Allan deviation (overlapping), implemented locally — no extra dependency.
# -----------------------------------------------------------------------------
def allan_deviation(y, dt):
    """Overlapping Allan deviation of a uniformly sampled series.
    Returns (taus [s], adev [same unit as y])."""
    y = np.asarray(y, dtype=float)
    n = len(y)
    taus, adev = [], []
    m = 1
    while m <= n // 4:
        a = np.convolve(y, np.ones(m) / m, mode="valid")  # length n-m+1
        d = a[m:] - a[:-m]
        if len(d) < 2:
            break
        taus.append(m * dt)
        adev.append(math.sqrt(0.5 * float(np.mean(d * d))))
        m = max(m + 1, int(round(m * 1.4)))               # ~log spacing
    return np.array(taus), np.array(adev)


def longest_settled_run(df):
    """Longest contiguous settled run (for Allan analysis). Returns the slice
    and its median sample interval, or (None, None)."""
    s = (df["settled"] == 1).astype(int)
    grp = (s.diff().fillna(0) != 0).cumsum()
    best = None
    for _, g in df.groupby(grp):
        if g["settled"].iloc[0] == 1 and (best is None or len(g) > len(best)):
            best = g
    if best is None or len(best) < ALLAN_MIN_PTS:
        return None, None
    dt = float(np.median(np.diff(best["t_s"].to_numpy())))
    return best, dt


# -----------------------------------------------------------------------------
# Plots (each returned as a base64 PNG for the self-contained HTML)
# -----------------------------------------------------------------------------
def gap_broken(t, y, factor=3.0):
    """Insert NaNs where the time axis jumps (device moving -> no rows), so
    line plots show gaps as gaps instead of bridging them with straight
    segments — without this a sparse log looks deceptively smooth."""
    t = np.asarray(t, dtype=float)
    y = np.asarray(y, dtype=float)
    if len(t) < 3:
        return t, y
    dt = np.diff(t)
    med = np.median(dt)
    idx = np.where(dt > factor * med)[0]
    if not len(idx):
        return t, y
    return np.insert(t, idx + 1, t[idx] + med), np.insert(y, idx + 1, np.nan)


def fig_to_b64(fig):
    buf = io.BytesIO()
    fig.savefig(buf, format="png", dpi=130, bbox_inches="tight",
                facecolor=fig.get_facecolor())
    plt.close(fig)
    return base64.b64encode(buf.getvalue()).decode("ascii")


def style_axes(ax):
    ax.grid(color=NOVA["outvar"], lw=0.6)
    for spine in ax.spines.values():
        spine.set_color(NOVA["outline"])
    ax.tick_params(colors=NOVA["onvar"], labelsize=8)
    ax.xaxis.label.set_color(NOVA["on"])
    ax.yaxis.label.set_color(NOVA["on"])
    ax.title.set_color(NOVA["on"])


def plot_timeseries(df, dwells, ranges):
    fig, (axp, axr, axt) = plt.subplots(3, 1, sharex=True, figsize=(11, 6.4),
                                        height_ratios=[3, 3, 1.4])
    fig.patch.set_facecolor(NOVA["card"])
    for ax, col, color, name in ((axp, "pitch_deg", NOVA["primary"], "pitch [deg]"),
                                 (axr, "roll_deg", NOVA["info"], "roll [deg]")):
        ax.plot(*gap_broken(df["t_s"], df[col]), color=color, lw=0.8)
        ax.set_ylabel(name)
        style_axes(ax)
        for _, dw in dwells.iterrows():
            ax.axvspan(dw["t0"], dw["t1"], color=NOVA["success"], alpha=0.08)
        for k, (a, b) in enumerate(ranges):
            ax.axvspan(a, b, color=SELECTION_COLORS[k % len(SELECTION_COLORS)], alpha=0.18)
    axt.plot(*gap_broken(df["t_s"], df["imu_temp_c"]), color=NOVA["warning"], lw=0.9)
    axt.set_ylabel("IMU [°C]")
    axt.set_xlabel("time since log start [s]")
    style_axes(axt)
    for _, e in df[df["event_flag"] == 1].iterrows():
        axp.axvline(e["t_s"], color=NOVA["brand"], lw=1)
        axp.annotate(str(e["event_label"]), (e["t_s"], axp.get_ylim()[1]),
                     fontsize=6.5, rotation=90, va="top", color=NOVA["brand"])
    for k, (a, b) in enumerate(ranges):
        axp.annotate(f"S{k+1}", ((a + b) / 2, axp.get_ylim()[0]), fontsize=9,
                     ha="center", va="bottom", fontweight="bold",
                     color=SELECTION_COLORS[k % len(SELECTION_COLORS)])
    axp.set_title("Time series — settled periods shaded green, selections shaded in colour, events flagged")
    return fig_to_b64(fig)


def conf_ellipse(ax, x, y, nsig, color, ls):
    if len(x) < 3:
        return
    cov = np.cov(x, y)
    if not np.all(np.isfinite(cov)):
        return
    vals, vecs = np.linalg.eigh(cov)
    vals = np.maximum(vals, 0)
    order = vals.argsort()[::-1]
    vals, vecs = vals[order], vecs[:, order]
    ang = math.degrees(math.atan2(vecs[1, 0], vecs[0, 0]))
    w, h = 2 * nsig * np.sqrt(vals)
    ax.add_patch(Ellipse((np.mean(x), np.mean(y)), w, h, angle=ang,
                         fill=False, color=color, ls=ls, lw=1.1))


def plot_2d(df, results, ranges):
    fig, ax = plt.subplots(figsize=(7.4, 7.4))
    fig.patch.set_facecolor(NOVA["card"])
    style_axes(ax)
    ax.axhline(0, color=NOVA["outvar"], lw=0.8)
    ax.axvline(0, color=NOVA["outvar"], lw=0.8)
    ax.add_patch(Circle((0, 0), TARGET_DEG, fill=False, color=NOVA["success"],
                        lw=1.4, label=f"target ±{TARGET_DEG:g}°"))
    lim = TARGET_DEG * 1.4
    for k, ((a, b), res) in enumerate(zip(ranges, results)):
        if not res.get("valid"):
            continue
        sel = df[(df["t_s"] >= a) & (df["t_s"] <= b)]
        used = sel[sel["settled"] == 1] if res["settled_only"] else sel
        col = SELECTION_COLORS[k % len(SELECTION_COLORS)]
        x, y = used["roll_deg"].to_numpy(), used["pitch_deg"].to_numpy()
        ax.scatter(x, y, s=4, color=col, alpha=0.30, lw=0)
        ax.scatter([np.mean(x)], [np.mean(y)], s=70, color=col, marker="+",
                   lw=1.8, label=f"S{k+1} mean (n={len(x)})")
        conf_ellipse(ax, x, y, 1, col, "-")
        conf_ellipse(ax, x, y, 2, col, "--")
        dsel = res.get("dwell_table")
        if dsel is not None and len(dsel):
            ax.scatter(dsel["roll_mean"], dsel["pitch_mean"], s=26, marker="o",
                       facecolors="none", edgecolors=col, lw=1.2)
        lim = max(lim, np.percentile(np.abs(x), 99.5) * 1.3,
                  np.percentile(np.abs(y), 99.5) * 1.3)
    ax.set_xlim(-lim, lim)
    ax.set_ylim(-lim, lim)
    ax.set_aspect("equal")
    ax.set_xlabel("roll [deg]")
    ax.set_ylabel("pitch [deg]")
    sx = ax.secondary_xaxis("top", functions=(
        lambda d: np.tan(np.radians(d)) * PLATE_ALONG_ROLL_MM * 1000.0,
        lambda u: np.degrees(np.arctan(u / (PLATE_ALONG_ROLL_MM * 1000.0)))))
    sy = ax.secondary_yaxis("right", functions=(
        lambda d: np.tan(np.radians(d)) * PLATE_ALONG_PITCH_MM * 1000.0,
        lambda u: np.degrees(np.arctan(u / (PLATE_ALONG_PITCH_MM * 1000.0)))))
    sx.set_xlabel(f"displacement over {PLATE_ALONG_ROLL_MM:g} mm [µm]", fontsize=8, color=NOVA["onvar"])
    sy.set_ylabel(f"displacement over {PLATE_ALONG_PITCH_MM:g} mm [µm]", fontsize=8, color=NOVA["onvar"])
    sx.tick_params(colors=NOVA["onvar"], labelsize=7)
    sy.tick_params(colors=NOVA["onvar"], labelsize=7)
    ax.legend(loc="upper right", fontsize=8, framealpha=0.9)
    ax.set_title("2-D tilt — samples, dwell means (○), selection means (+), 1σ/2σ ellipses",
                 fontsize=10)
    return fig_to_b64(fig)


def plot_allan(run, dt):
    taus_p, ad_p = allan_deviation(run["pitch_deg"].to_numpy(), dt)
    taus_r, ad_r = allan_deviation(run["roll_deg"].to_numpy(), dt)
    fig, ax = plt.subplots(figsize=(7.6, 4.6))
    fig.patch.set_facecolor(NOVA["card"])
    ax.loglog(taus_p, ad_p, "o-", ms=3, lw=1.1, color=NOVA["primary"], label="pitch")
    ax.loglog(taus_r, ad_r, "o-", ms=3, lw=1.1, color=NOVA["info"], label="roll")
    if len(ad_p):
        i = int(np.argmin(ad_p))
        ax.annotate(f"floor ≈ {ad_p[i]*1000:.2f} m° @ τ={taus_p[i]:.0f} s",
                    (taus_p[i], ad_p[i]), textcoords="offset points",
                    xytext=(8, -12), fontsize=8, color=NOVA["primary"])
    style_axes(ax)
    ax.set_xlabel("averaging time τ [s]")
    ax.set_ylabel("Allan deviation [deg]")
    ax.legend(fontsize=9)
    ax.set_title(f"Allan deviation — longest settled run "
                 f"({len(run)} pts @ {dt:.1f} s)", fontsize=10)
    ax.grid(which="minor", color=NOVA["outvar"], lw=0.4, alpha=0.6)
    return fig_to_b64(fig), (taus_p, ad_p, taus_r, ad_r)


# -----------------------------------------------------------------------------
# HTML report
# -----------------------------------------------------------------------------
CSS = f"""
body{{background:{NOVA['surface']};color:{NOVA['on']};max-width:900px;margin:0 auto;
 font:14px/1.5 system-ui,-apple-system,'Segoe UI',Roboto,Arial,sans-serif;padding:18px}}
h1{{font-size:21px;letter-spacing:.04em}} h2{{font-size:15px;margin:26px 0 8px;
 text-transform:uppercase;letter-spacing:.1em;color:{NOVA['onvar']}}}
.brand{{height:4px;background:{NOVA['brand']};width:80px;border-radius:2px;margin:8px 0 14px}}
.card{{background:{NOVA['card']};border:1px solid {NOVA['outvar']};border-radius:12px;
 padding:14px;margin:12px 0}}
img{{max-width:100%;border-radius:8px}}
table{{border-collapse:collapse;width:100%;font-size:12.5px}}
th,td{{border-bottom:1px solid {NOVA['outvar']};padding:4px 8px;text-align:right;
 font-variant-numeric:tabular-nums}}
th{{color:{NOVA['onvar']};font-weight:600;text-transform:uppercase;font-size:10.5px;
 letter-spacing:.06em}}
td:first-child,th:first-child{{text-align:left}}
.note{{font-size:12px;color:{NOVA['onvar']}}}
.ok{{color:{NOVA['success']};font-weight:700}} .warn{{color:{NOVA['warning']};font-weight:700}}
.kv td{{text-align:left}}
"""


def fmt(v, nd=3, suf=""):
    return "–" if v is None or (isinstance(v, float) and not np.isfinite(v)) else f"{v:+.{nd}f}{suf}"


def selection_table_html(results):
    rows = []
    for k, r in enumerate(results):
        if not r.get("valid"):
            rows.append(f"<tr><td>S{k+1}</td><td colspan=11>too few settled samples "
                        f"({r['n_used']}/{r['n_total']})</td></tr>")
            continue
        for axn in ("pitch", "roll"):
            a = r[axn]
            rows.append(
                "<tr>"
                + (f"<td rowspan=2><b>S{k+1}</b><br><span class=note>{r['t0']:.0f}–{r['t1']:.0f} s<br>"
                   f"n={r['n_used']}</span></td>" if axn == "pitch" else "")
                + f"<td>{axn}</td>"
                  f"<td>{fmt(a['mean'])}</td><td>{a['std']*1000:.2f}</td>"
                  f"<td>{a['pp']*1000:.2f}</td><td>{fmt(a['min'])}</td><td>{fmt(a['max'])}</td>"
                  f"<td>{fmt(a['drift_deg_min']*1000, 2)}</td>"
                  f"<td>{fmt(a['mean_um'], 1)}</td><td>{a['std_um']:.1f}</td><td>{a['pp_um']:.1f}</td>"
                "</tr>")
    return ("<table><tr><th>sel</th><th>axis</th><th>mean [°]</th><th>σ [m°]</th>"
            "<th>p-p [m°]</th><th>min [°]</th><th>max [°]</th><th>drift [m°/min]</th>"
            "<th>mean [µm]</th><th>σ [µm]</th><th>p-p [µm]</th></tr>"
            + "".join(rows) + "</table>"
            "<p class=note>µm columns: height displacement across the plate "
            f"({PLATE_ALONG_PITCH_MM:g} mm for pitch, {PLATE_ALONG_ROLL_MM:g} mm for roll).</p>")


def build_report(args, df, sessions, dwells, ranges, results, imgs, allan_info):
    title = args.title or "Static Level Logger — evaluation report"
    t_first = df["wall"].dropna()
    span_wall = (f"{t_first.iloc[0]:%Y-%m-%d %H:%M:%S} … {t_first.iloc[-1]:%H:%M:%S}"
                 if len(t_first) else "no wall clock (NO_TIME) — using millis")
    ox, oy = recovered_offsets(df[df["calibrated"] == 1]) if (df["calibrated"] == 1).any() \
        else (pd.Series(dtype=float), pd.Series(dtype=float))
    cal_txt = (f"offsets active: X {ox.mean()*1000:+.2f} mg / Y {oy.mean()*1000:+.2f} mg "
               f"(≈ {abs(ox.mean()*1000)*DEG_PER_MG:.3f}° / {abs(oy.mean()*1000)*DEG_PER_MG:.3f}°)"
               if len(ox) else "log contains UNCALIBRATED data (zero offsets)")
    if 0 < (df["calibrated"] == 1).sum() < len(df):
        cal_txt += " — <span class=warn>calibration state CHANGED during the log</span>"

    meta_rows = "".join(
        f"<tr><td>{k}</td><td>{v}</td></tr>" for k, v in [
            ("Source file(s)", ", ".join(f"{n} ({c} rows)" for n, c in sessions)),
            ("Wall-clock span", span_wall),
            ("Duration", f"{df['t_s'].iloc[-1]:.0f} s"),
            ("Rows / settled rows", f"{len(df)} / {(df['settled'] == 1).sum()}"),
            ("Dwells (≥ {:.0f} s)".format(MIN_DWELL_S), str(len(dwells))),
            ("Calibration", cal_txt),
            ("IMU temperature", f"{df['imu_temp_c'].min():.1f} … {df['imu_temp_c'].max():.1f} °C"),
            ("Statistics basis", "settled rows only" if results and results[0]["settled_only"]
             else "all rows in range"),
            ("Mounting", PLATE_NOTE),
            ("Generated", _dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S")),
        ])

    repeat_html = ""
    for k, r in enumerate(results):
        if r.get("valid") and r.get("n_dwells", 0) >= 2:
            repeat_html += (
                f"<p>S{k+1}: between-dwell repeatability over {r['n_dwells']} dwells — "
                f"pitch σ <b>{r['repeat_pitch_std']*1000:.2f} m°</b> "
                f"({disp_um(r['repeat_pitch_std'], PLATE_ALONG_PITCH_MM):.1f} µm), "
                f"roll σ <b>{r['repeat_roll_std']*1000:.2f} m°</b> "
                f"({disp_um(r['repeat_roll_std'], PLATE_ALONG_ROLL_MM):.1f} µm)</p>")
    if not repeat_html:
        repeat_html = ("<p class=note>No selection contains ≥ 2 dwells — re-place the plate "
                       "several times inside one selection to evaluate mounting repeatability.</p>")

    dwell_html = ""
    if not dwells.empty:
        d = dwells.copy()
        d["σp m°"] = (d["pitch_std"] * 1000).round(2)
        d["σr m°"] = (d["roll_std"] * 1000).round(2)
        d = d[["dwell", "t0", "dur_s", "n", "pitch_mean", "σp m°", "roll_mean", "σr m°", "temp_mean"]]
        d.columns = ["#", "start [s]", "dur [s]", "n", "pitch [°]", "σp [m°]",
                     "roll [°]", "σr [m°]", "IMU [°C]"]
        dwell_html = d.to_html(index=False, float_format=lambda v: f"{v:.3f}")

    allan_html = ""
    if allan_info:
        img, (tp, ap_, tr, ar_) = allan_info
        floor_p = (ap_.min() * 1000) if len(ap_) else float("nan")
        floor_r = (ar_.min() * 1000) if len(ar_) else float("nan")
        allan_html = f"""
<h2>Allan deviation (stability vs averaging time)</h2>
<div class=card><img src="data:image/png;base64,{img}">
<p class=note>Computed from the longest contiguous settled run. Stability floor:
pitch ≈ <b>{floor_p:.2f} m°</b>, roll ≈ <b>{floor_r:.2f} m°</b>. The logged rows are
already {'' if True else ''}overlapping window averages from the firmware
(≈ 5 s default), so values below τ ≈ the averaging window are smoothed by design;
read the curve for τ above it. A falling curve means longer observation still
helps; a rising tail reveals drift (temperature) dominating.</p></div>"""

    events_html = ""
    evs = [(k, e) for k, r in enumerate(results) if r.get("valid") for e in r["events"]]
    if evs:
        events_html = ("<h2>Events in selections</h2><div class=card><table>"
                       "<tr><th>sel</th><th>t [s]</th><th>label</th></tr>"
                       + "".join(f"<tr><td>S{k+1}</td><td>{t:.1f}</td><td>{lbl}</td></tr>"
                                 for k, (t, lbl) in evs) + "</table></div>")

    html = f"""<!DOCTYPE html><html><head><meta charset="utf-8">
<title>{title}</title><style>{CSS}</style></head><body>
<h1>{title}</h1><div class=brand></div>

<h2>Session</h2>
<div class=card><table class=kv>{meta_rows}</table></div>

<h2>Time series &amp; selections</h2>
<div class=card><img src="data:image/png;base64,{imgs['ts']}"></div>

<h2>2-D tilt / displacement</h2>
<div class=card><img src="data:image/png;base64,{imgs['2d']}">
<p class=note>Primary axes in degrees (equal aspect); secondary axes show the height
displacement the tilt produces across the plate footprint. Target circle:
±{TARGET_DEG:g}° ({disp_um(TARGET_DEG, PLATE_ALONG_PITCH_MM):.0f} µm over
{PLATE_ALONG_PITCH_MM:g} mm / {disp_um(TARGET_DEG, PLATE_ALONG_ROLL_MM):.0f} µm over
{PLATE_ALONG_ROLL_MM:g} mm).</p></div>

<h2>Selection statistics</h2>
<div class=card>{selection_table_html(results)}</div>

<h2>Repeatability (between dwells)</h2>
<div class=card>{repeat_html}</div>

<h2>Dwell inventory</h2>
<div class=card>{dwell_html or '<p class=note>no dwells found</p>'}</div>

{allan_html}
{events_html}

<h2>Notes</h2>
<div class=card class=note><ul class=note>
<li>σ figures describe <b>precision</b> (noise/repeatability). The <b>absolute zero
accuracy</b> can only be judged against a flip measurement (0°/180° pair) or an
external reference — not included in this report's scope.</li>
<li>Recovered calibration offsets come from the logged raw−corrected columns.</li>
<li>{PLATE_NOTE}</li>
</ul></div>
</body></html>"""
    return html


# -----------------------------------------------------------------------------
# Main
# -----------------------------------------------------------------------------
# --- file selection for argument-less launches (IDE 'Run', double-click) -----
# Three mechanisms, tried in order. Each returns None when the MECHANISM is
# unavailable (try the next one) and [] when the USER cancelled (stop asking).

def _pick_tkinter():
    try:
        import tkinter as tk
        from tkinter import filedialog
        root = tk.Tk()
        root.withdraw()
        root.update()
        files = filedialog.askopenfilenames(
            title="Select level_log CSV file(s)", initialdir=os.getcwd(),
            filetypes=[("Level logger CSV", "level_log_*.csv"),
                       ("CSV files", "*.csv"), ("All files", "*.*")])
        root.destroy()
        return list(files)
    except Exception:
        return None                       # tkinter missing/headless -> next mechanism


def _pick_powershell():
    """Native Windows open-file dialog without tkinter (System.Windows.Forms)."""
    if sys.platform != "win32":
        return None
    ps = ("Add-Type -AssemblyName System.Windows.Forms;"
          "$d=New-Object System.Windows.Forms.OpenFileDialog;"
          "$d.Multiselect=$true;"
          "$d.InitialDirectory='" + os.getcwd().replace("'", "''") + "';"
          "$d.Filter='Level logger CSV (level_log_*.csv)|level_log_*.csv"
          "|CSV files (*.csv)|*.csv|All files (*.*)|*.*';"
          "$d.Title='Select level_log CSV file(s)';"
          "if($d.ShowDialog() -eq 'OK'){$d.FileNames -join [Environment]::NewLine}")
    try:
        out = subprocess.run(["powershell", "-NoProfile", "-STA", "-Command", ps],
                             capture_output=True, text=True, timeout=600)
        if out.returncode != 0:
            return None
        return [l.strip() for l in out.stdout.splitlines() if l.strip()]
    except Exception:
        return None


def _pick_console():
    """Last resort: list level_log_*.csv found near the script / cwd and let
    the user pick by number, or type a path. Works everywhere, even over SSH."""
    import glob
    here = os.path.dirname(os.path.abspath(__file__))
    dirs = [os.getcwd(), here, os.path.dirname(here)]
    cands, seen = [], set()
    for d in dirs:
        for f in sorted(glob.glob(os.path.join(d, "level_log_*.csv"))):
            rp = os.path.realpath(f)
            if rp not in seen:
                seen.add(rp)
                cands.append(f)
    try:
        if not cands:
            path = input("Path to a level_log CSV: ").strip().strip('"')
            return [path] if path else []
        print("\nFound log files:")
        for i, c in enumerate(cands, 1):
            print(f"  [{i}] {c}  ({os.path.getsize(c)//1024} KB)")
        sel = input("Select number(s), comma-separated — or type a path: ").strip()
        if not sel:
            return []
        if all(p.strip().isdigit() for p in sel.split(",")):
            idx = [int(p) for p in sel.split(",")]
            return [cands[i - 1] for i in idx if 1 <= i <= len(cands)]
        return [sel.strip('"')]
    except EOFError:
        return []


def pick_files_dialog():
    for mech in (_pick_tkinter, _pick_powershell):
        r = mech()
        if r is not None:                 # mechanism worked ([] = user cancelled)
            return r
    return _pick_console()


def main():
    ap = argparse.ArgumentParser(description="Evaluate Static Level Logger CSVs into an HTML report.")
    ap.add_argument("logs", nargs="*", help="level_log_*.csv file(s); "
                    "omit to choose via a file dialog")
    ap.add_argument("-o", "--out", help="output HTML path")
    ap.add_argument("--all", action="store_true", help="whole log as one selection (no GUI)")
    ap.add_argument("--reuse", action="store_true", help="reuse saved selections (no GUI)")
    ap.add_argument("--title", default=None)
    ap.add_argument("--include-moving", action="store_true",
                    help="include settled=0 rows in statistics (default: settled only)")
    args = ap.parse_args()

    from_picker = False
    if not args.logs:                      # launched without arguments (IDE/double-click)
        print("No log file given — opening a file picker ...")
        args.logs = pick_files_dialog()
        from_picker = bool(args.logs)
        if not args.logs:
            ap.print_help()
            raise SystemExit("\nNo log selected. Pass the CSV path, e.g.:\n"
                             "  python tools/evaluate_level_log.py level_log_0001.csv")

    df, sessions = load_logs(args.logs)
    dwells = segment_dwells(df)
    print(f"Loaded {len(df)} rows ({(df['settled']==1).sum()} settled), "
          f"{len(dwells)} dwells ≥ {MIN_DWELL_S:.0f} s.")

    sidecar = selections_sidecar_path(args.logs[0])
    ranges = None
    if args.all:
        ranges = [(float(df["t_s"].iloc[0]), float(df["t_s"].iloc[-1]))]
    elif args.reuse:
        if not os.path.exists(sidecar):
            raise SystemExit(f"--reuse: no saved selections at {sidecar}")
        ranges = [tuple(r) for r in json.load(open(sidecar))["ranges"]]
        print(f"Reusing {len(ranges)} selection(s) from {sidecar}")
    else:
        try:
            ranges = select_ranges_interactive(df, dwells)
        except Exception as e:
            print(f"Interactive selection unavailable ({e}); falling back to the whole log. "
                  f"Use --all to silence this.")
        if not ranges:
            ranges = [(float(df["t_s"].iloc[0]), float(df["t_s"].iloc[-1]))]
            print("No ranges selected — using the whole log.")
        else:
            json.dump({"source": os.path.basename(args.logs[0]), "ranges": ranges},
                      open(sidecar, "w"), indent=1)
            print(f"Selections saved to {sidecar} (re-run with --reuse).")

    settled_only = not args.include_moving
    results = [eval_selection(df, dwells, a, b, settled_only) for a, b in ranges]

    imgs = {"ts": plot_timeseries(df, dwells, ranges),
            "2d": plot_2d(df, results, ranges)}
    run, dt = longest_settled_run(df)
    allan_info = plot_allan(run, dt) if run is not None else None
    if allan_info is None:
        print(f"Allan deviation skipped: no contiguous settled run ≥ {ALLAN_MIN_PTS} points.")

    out = args.out or os.path.splitext(args.logs[0])[0] + "_report.html"
    open(out, "w", encoding="utf-8").write(
        build_report(args, df, sessions, dwells, ranges, results, imgs, allan_info))
    print(f"Report written: {out}")
    if from_picker:                        # GUI-launched -> show the result directly
        import webbrowser
        webbrowser.open("file://" + os.path.abspath(out))

    for k, r in enumerate(results):                      # console summary
        if r.get("valid"):
            print(f"  S{k+1}: pitch {r['pitch']['mean']:+.3f}° σ{r['pitch']['std']*1000:.2f} m° "
                  f"({r['pitch']['std_um']:.1f} µm) | roll {r['roll']['mean']:+.3f}° "
                  f"σ{r['roll']['std']*1000:.2f} m° ({r['roll']['std_um']:.1f} µm) | n={r['n_used']}")


if __name__ == "__main__":
    main()
