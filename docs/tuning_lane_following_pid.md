# Tuning the PID Controller for lane following

## 1. Increase $K_p$ by 0.5 until Duckiebot overshoots multiple times coming out of corners
- $K_P$ for Tick: **11.5**

## 2. Increase $K_D$ by 0.1 until straigth-line wobbeling smooths out
- $K_D$ for Tick: **0.2 - 1.4** (not much change)

## 3. Add a little $K_I$ (much less than $K_P$ and $K_D$)
- $K_I$ for Tick: **0.005**