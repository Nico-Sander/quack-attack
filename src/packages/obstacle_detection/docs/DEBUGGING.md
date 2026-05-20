# Debugging und typische Fehlerbilder

Dieses Dokument beschreibt, wo man bei typischen Problemen zuerst suchen sollte.

---

## Der Bot fährt nicht los

Prüfen:

```bash
echo $VEHICLE_NAME
rostopic hz /$VEHICLE_NAME/detect/lane
rostopic echo /$VEHICLE_NAME/car_cmd_switch_node/cmd
```

Wenn `/detect/lane` keine Werte liefert, läuft der Lane-Node nicht korrekt oder im falschen Namespace.

Wenn `car_cmd_switch_node/cmd` keine Werte liefert, wird `cbFollowLane()` nicht getriggert oder das Topic ist falsch.

---

## Duckie wird als gelbe oder weiße Linie erkannt

Prüfen:

```bash
rostopic echo /$VEHICLE_NAME/detect/lane_borders
```

Wichtig sind:

```text
yellow_valid
white_valid
```

Im Dashboard prüfen:

```text
Yellow mask
White mask
```

Die Duckie-Bereiche sollten dort aus der Maske entfernt sein.

---

## Rote Sperrbereiche flackern

Ursachen:

```text
YOLO erkennt Duckie nicht in jedem Frame
Confidence schwankt
Duckie ist noch zu klein
```

Relevante Parameter:

```text
duckie_hold_time
duckie_missed_frames_before_clear
confidence_threshold
min_duckie_area_px
```

---

## Bot reagiert zu spät auf Duckies

Relevante Parameter:

```text
y_min
min_duckie_width_px
min_duckie_height_px
min_duckie_area_px
lane_target_block_margin
```

`y_min` kleiner machen bedeutet: Duckies werden weiter oben im Bild relevant.

---

## Bot reagiert zu früh auf entfernte Duckies

Relevante Parameter:

```text
min_duckie_width_px
min_duckie_height_px
min_duckie_area_px
```

Diese Werte erhöhen, wenn entfernte Duckies zu früh beeinflussen.

---

## Bot fährt zu knapp an Duckies vorbei

Relevante Parameter:

```text
duckie_x_margin
escape_clearance
min_free_width_px
```

Mehr Sicherheitsabstand erzeugt robustere Umfahrungen, kann aber freie Bereiche zu klein machen.

---

## Bot fährt beim starken Ausweichen zu weit vorwärts

Wenn das Ziel weit links oder rechts liegt, sollte `avoidance_turn_in_place` aktiv werden.

Relevante Parameter:

```text
avoidance_turn_in_place_error_enter
avoidance_turn_in_place_error_exit
avoidance_turn_in_place_omega
avoidance_vel
```

`avoidance_vel` reduziert die Vorwärtsfahrt während der Ausweichphase.

---

## Bot bleibt mit `no_valid_escape_target` stehen

Dann findet der Controller keine gültige freie Lücke.

Prüfen im Dashboard:

```text
reason
blocked_intervals
free_intervals
selected_free_interval
left_open / right_open
```

Relevante Parameter:

```text
min_free_width_px
open_side_width_bonus
blocked_recovery_delay
blocked_recovery_omega
blocked_recovery_min_turn_time
blocked_recovery_max_angle_deg
```

---

## Dashboard crasht mit X11/MIT-SHM-Fehler

Starten mit:

```bash
QT_X11_NO_MITSHM=1 rosrun obstacle_detection dashboard_node.py
```

Optional:

```bash
export QT_X11_NO_MITSHM=1
export NO_AT_BRIDGE=1
```
