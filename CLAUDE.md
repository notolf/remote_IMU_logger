# remote_IMU_logger

M5Stack CoreS3 static level logger (BMI270 IMU): firmware under `src/`,
offline evaluation tooling under `tools/`.

## Design doctrine

All user-facing output — HTML reports, dashboards, plots, GUIs — follows
the **classic Leica** design doctrine in `docs/DESIGN.md` (white/graphite/
hairline greys, Leica red `#DA291C` as the only accent, Helvetica stack,
logo slot at `tools/assets/leica_logo.*`). This is a deliberate owner
decision and supersedes the Hexagon Nova design system for this project
family. Do not introduce new colours or fonts outside the doctrine;
`tools/evaluate_level_log.py` (`LEICA` dict, `CSS` block) is the reference
implementation.

Non-negotiable visualisation rules: show data gaps as gaps, show
measurement uncertainty (R95 circle of confusion) at a visible scale,
and flag under-sampled measurements (< 30 s or n < 20 by default).
