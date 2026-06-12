# Design Doctrine — Classic Leica

Binding style guide for **every user-facing artifact** in this and future
projects: HTML reports, dashboards, plots, GUIs, documents. Decided
2026-06-12 (project owner). It supersedes the Hexagon **Nova** design system
for this project family — Nova remains the corporate DS, but these tools
deliberately use the classic Leica look.

## 1. Identity

- **Logo**: the official Leica Geosystems logo (red script wordmark).
  Tools embed the asset from `tools/assets/leica_logo.svg|png|jpg` when
  present and fall back to a typographic wordmark otherwise — never
  redraw, recolour, stretch, or approximate the script lettering.
  Obtain the asset from the internal brand portal. Think before
  committing it to a public repository; the embed-from-disk slot exists
  so the binary never *needs* to be committed.
- Logo placement: top-right of the header, ~48–56 px tall, generous
  clear space, always on white.
- A full-width 3 px **Leica red rule** closes the header.

## 2. Colour

White, graphite, hairline grey — and Leica red as *the* accent. Red is
spent on identity and the most important figure, not decoration.

| Role | Hex | Usage |
|---|---|---|
| Leica red (Pantone 485 C) | `#DA291C` | brand accents, header rule, primary data series, S1 |
| Graphite (primary text) | `#1A1A1A` | body text, secondary data series |
| Secondary text | `#6E6E6E` | captions, labels, table headers |
| Page / cards | `#FFFFFF` | all surfaces |
| Recessed fill | `#F7F7F7` | subtle background fills |
| Hairline rule | `#E6E6E6` | borders, dividers, grids |
| Strong rule | `#B3B3B3` | axes, emphasised borders |
| Success | `#2E7D32` (bg `#E9F2E9`, text `#2C5E2E`) | pass states, target markers |
| Warning | `#C77B00` (bg `#FBF0DC`, text `#8A5800`) | sample-size/duration flags, temperature trace |
| Error | `#A81F15` | failures — darker than brand red so alarms read as alarms |
| Categorical series | `#DA291C` `#1A1A1A` `#3A6EA5` `#2E7D32` `#7D4B9E` `#C77B00` `#2E7D7B` `#8E5515` | selections, multi-series plots (red first) |

## 3. Typography

- Stack: `"Helvetica Neue", Helvetica, Arial, sans-serif` — regular and
  bold only, no italics for emphasis.
- Scale: title 23/34 bold · section 16/24 bold (sentence case) ·
  body 14/20 · labels & captions 12/18 · table data 12.5 with
  `font-variant-numeric: tabular-nums`.
- Overline labels: 12 px bold, uppercase, letter-spaced, **Leica red**.

## 4. Layout & components

- Single column, max-width 880 px, generous whitespace.
- **Hero first**: the key figures (means, uncertainty, status) as stat
  tiles before any plot. A reader gets the verdict without scrolling.
- Cards: white, 1 px hairline border, 12 px radius, 12 px padding.
  No shadows.
- Status chips: pill, tinted container background (`okbg`/`warnbg`),
  bold 12 px text.
- Detail belongs in collapsed `<details>` sections (inventories, method
  notes, secondary analyses). The default view stays sleek.

## 5. Data visualisation rules

- Series colours: pitch = Leica red, roll = graphite, temperature =
  muted amber. White plot background, hairline grids.
- **Gap honesty**: never bridge time gaps with interpolated lines —
  insert NaNs so missing data shows as missing.
- **Uncertainty is always visible**: every measurement aggregate carries
  a circle of confusion (R95). If the overview scale makes it sub-pixel,
  per-selection detail panels at true scale are mandatory. Never inflate
  an indicator to make it visible.
- **Sample-size guards**: measurements below the required duration
  (default 30 s) or below n = 20 settled samples are flagged everywhere
  they appear (console, hero chip, tables, plot legends). Plots use an
  ASCII `*` for the flag — unicode ⚠ renders as tofu in common
  matplotlib font setups; HTML may use ⚠.
- Target/tolerance circles in success green; σ describes precision,
  never absolute accuracy — say so in the fine print.

## 6. Report anatomy (reference implementation)

`tools/evaluate_level_log.py` is the canonical implementation of this
doctrine: header (overline · title · logo · red rule) → hero tiles →
time series → 2-D tilt with CoC + detail panels → selection table →
collapsed details → footer. New artifacts copy this structure and this
file's `LEICA` palette dict verbatim.

For a new project: copy this file to `docs/DESIGN.md`, reference it from
the project's `CLAUDE.md`, and copy the `LEICA` dict + `CSS` block from
`tools/evaluate_level_log.py` as the starting point.
