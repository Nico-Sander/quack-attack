# Parameter migration: old controller → lean follow-the-gap

Maps the 47 parameters of the old `control_lane_node` (branch
`feature/obstacle-detection`) onto the 19 of this one.

The two controllers do not decompose the problem the same way, so this is not a
rename table. Roughly a third of the old knobs carry over directly, a third need
converting, and a third describe machinery that no longer exists. Four knobs here
are new and have no old value to inherit.

**None of the suggested values have been driven.** They are conversions of your
tuned numbers, not measurements. Treat the "suggested" column as a starting point
that should still be checked on the robot.

---

## 1. Direct carries

Same meaning, same units. Copy the value.

| old | your value | new | default | suggested |
|---|---|---|---|---|
| `pid.p` | 6.0 | `k_steer` | 6.0 | **6.0** — already identical; `k_steer` *is* the lane P-gain |
| `lane_margin` | 0.08 | `lane_margin` | 0.12 | **0.08** |
| `duckie_hold_time` | 0.8 | `duckie_hold_time` | 1.0 | **0.8** |
| `min_vel` | 0.05 | `v_min` | 0.04 | **0.05** |
| `y_min` | 0.6 | `react_ymax` | 0.55 | **0.6** — same idea: closeness at which a duckie starts to matter |

## 2. Needs conversion

| old | your value | new | conversion | suggested |
|---|---|---|---|---|
| `max_vel` | 0.15 | `v_cruise` | cruise speed, was the PID's ceiling | **0.15** |
| `avoidance_vel` | 0.15 | `v_avoid` | you had avoid == cruise | **0.15**, but see note below |
| `max_omega` | 5.0 | `omega_max` | same cap | **5.0** — raises the new default of 3.0; see note |
| `avoidance_turn_in_place_omega` | 1.0 (2.0 uncommitted) | `omega_rotate` | in-place rotation rate | **2.0** — floored at `MIN_ESCAPE_OMEGA` = 0.8 |
| `blocked_recovery_omega` | 2 | `omega_rotate` | same knob in the new design | consistent with 2.0 above |
| `min_free_width_px` / `planner_image_width_px` | 40 / 192 | `gap_min_width` | 40 ÷ 192 = 0.208 | **0.21** — much stricter than the 0.1 default; keep `front_slice_half` < 0.104 |
| `duckie_x_margin` | 0.1 | `duckie_margin_base` + `duckie_margin_gain` | old inflation was fixed; new is `base + gain × proximity` | **base 0.04, gain 0.10** → 0.04 far, 0.14 close. For literally the old behaviour: base 0.1, gain 0.0 |
| `min_duckie_width_px` / `min_duckie_height_px` | 30 / 30 of 640×480 | `MIN_DUCKIE_WIDTH` / `MIN_DUCKIE_HEIGHT` (module constants) | 30÷640 = 0.047, 30÷480 = 0.063 | constants are 0.04 / 0.04 — slightly more permissive. Edit in source if it matters |
| `blocked_recovery_delay` | 1.5 | `escape_relax_after` | time rotating before margins relax | **1.5** |
| `avoidance_side_lock_time` | 0.4 | `avoid_min_dwell` | commitment window | keep **0.8** — the new default is deliberately longer, see `PARAMETERS.md` |

## 3. No equivalent — machinery that no longer exists

Nothing to port. Listed so you can confirm nothing was lost silently.

| old | why it is gone |
|---|---|
| `avoidance_kp`, `avoidance_ki`, `avoidance_kd` | one steering gain (`k_steer`) for all states; no separate avoidance PID |
| `avoidance_steering_gain` | folded into `k_steer` (your effective avoidance gain was 4.5 × 0.8 = 3.6 against a lane gain of 6.0) |
| `pid.i`, `pid.d` | both were 0.0 / 1.0; the new steer law is proportional only |
| `narrow_gap_behavior: "stop"` | the never-freeze invariant forbids stopping; it rotates instead |
| `blocked_recovery_min_turn_time` (3), `min_angle_deg` (60), `max_angle_deg` (180) | the scan/return recovery is replaced by `ESCAPE_ROTATE` + `escape_relax_after`. **Do not port 3 s into `escape_min_dwell`** — at 2.0 rad/s that sweeps ~344°, and the lane lines leave the frame long before the state can be re-evaluated |
| `avoidance_clear_hold_time` (0.2) | now `CLEAR_HOLD_TIME` = 0.6, a module constant |
| `avoidance_side_lock_bonus` (0.2) | now `HYSTERESIS_MARGIN` 0.10 + `GAP_STICKY_BONUS` 0.06 |
| `escape_clearance` (0.04) | now `GAP_INSET` = 0.04 — identical value, now a constant |
| `default_lane_left/right` (0.05/0.95) | same values, now the planner's initial wall state |
| `lane_timeout` (1.0; 0.5 uncommitted) | now `LANE_TIMEOUT` = 1.0, a module constant. Your 0.5 has no config equivalent — edit the constant |
| `obstacle_timeout` (1.2) | superseded by `duckie_hold_time` |
| `duckie_missed_frames_before_clear` (4) | duckie staleness is purely time-based now; `lane_hold_frames` is the analogous debounce for *lines* |
| `y_max` (0.99) | now `PLAN_Y_MAX` = 1.0, a constant |
| `min_duckie_area_px` (950) | filtering is width/height only |
| `planner_image_width_px`, `obstacle_image_width_px`, `obstacle_image_height_px` | everything is normalized [0,1]; no pixel dimensions anywhere |
| `lane_target_block_margin`, `gap_width_bonus_weight`, `avoidance_target_smoothing_alpha`, `open_side_width_bonus`, `duckie_y_margin` | belong to the old scoring/smoothing scheme, which the gap planner replaces |

## 4. New — no old value to inherit

Tune these fresh; they are the ones most likely to need robot time.

| new | default | what it does |
|---|---|---|
| `front_slow_ymax` | 0.72 | closeness at which forward speed starts ramping toward `v_min` |
| `front_block_ymax` | 0.88 | closeness at which the front counts as blocked → `ESCAPE_ROTATE` |
| `front_slice_half` | 0.04 | half-width of the "something in my path" probe — the robot's own width. **Hard constraint: keep below `gap_min_width` / 2**, or every accepted gap self-aborts. The node warns at startup if violated |
| `escape_min_dwell` | 0.25 | minimum rotation before the escape state may be reconsidered |
| `lane_hold_frames` | 12 | invalid line readings before that side is treated as absent |

---

## Notes on three of the conversions

**`v_avoid` = 0.15.** You had avoidance at the same speed as cruise. This
controller inflates duckie margins with proximity, so a gap accepted at range
narrows as you approach; at 0.15 m/s there is less time for that to resolve
before `avoid_min_dwell` expires. If avoidance looks twitchy, drop `v_avoid`
first — it is the cheapest knob to try.

**`omega_max` = 5.0.** The new steer law is `k_steer × (0.5 − target_x) × 2`, so
with `k_steer` 6.0 a full-width error commands 6.0 rad/s and the cap actually
binds. The old controller reached the cap far less often. 5.0 is faithful to your
tuning but the 3.0 default exists because ≥4 is described as "very abrupt".

**`gap_min_width` = 0.21.** Notably stricter than the 0.1 default: you required
40 px of 192. With margins inflating on approach, a 0.21 requirement plus
`avoid_min_dwell` may reject gaps this controller could actually drive. Worth an
early A/B against 0.15.
