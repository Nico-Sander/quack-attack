# Parameterübersicht

Die Parameter liegen in:

```text
config/control_lane_node.json
config/detect_obstacle_node.json
```

Alle Werte in `control_lane_node.json` sind unter `parameters.path_planner` oder `parameters.pid` gruppiert.

---

## PID-Parameter für normales Lane-Following

| Parameter | Bedeutung |
|---|---|
| `p` | Proportionalanteil des normalen Lane-Following-Reglers. |
| `i` | Integralanteil des normalen Lane-Following-Reglers. |
| `d` | Differentialanteil des normalen Lane-Following-Reglers. |
| `max_vel` | Normale Vorwärtsgeschwindigkeit im Lane-Following. |

---

## Planungsfenster

| Parameter | Bedeutung |
|---|---|
| `y_min` | Oberer Rand des vertikalen Bildbereichs, in dem Duckies für die Planung relevant werden. |
| `y_max` | Unterer Rand des vertikalen Bildbereichs. |
| `duckie_y_margin` | Toleranz am oberen und unteren Rand des Planungsfensters. |

---

## Duckie-Sicherheitsbereiche

| Parameter | Bedeutung |
|---|---|
| `duckie_x_margin` | Seitlicher Sicherheitsabstand, der zur Duckie-Bounding-Box addiert wird. |
| `lane_target_block_margin` | Zusätzlicher Bereich um ein Duckie, in dem der normale Lane-Target als blockiert gilt. |
| `escape_clearance` | Abstand des Ausweichzielpunkts zum Rand eines blockierten Bereichs oder freien Intervalls. |
| `min_free_width_px` | Mindestbreite eines freien Fahrbereichs in Pixeln bezogen auf `planner_image_width_px`. |
| `planner_image_width_px` | Referenzbreite für pixelbasierte Breitenprüfungen. |

---

## Duckie-Filter und Hysterese

| Parameter | Bedeutung |
|---|---|
| `obstacle_image_width_px` | Referenzbreite der YOLO-Bounding-Boxes. |
| `obstacle_image_height_px` | Referenzhöhe der YOLO-Bounding-Boxes. |
| `min_duckie_width_px` | Mindestbreite einer Duckie-Box, damit sie berücksichtigt wird. |
| `min_duckie_height_px` | Mindesthöhe einer Duckie-Box. |
| `min_duckie_area_px` | Mindestfläche einer Duckie-Box. |
| `duckie_hold_time` | Zeit, für die ein zuletzt erkanntes Duckie gehalten wird. |
| `duckie_missed_frames_before_clear` | Anzahl verpasster YOLO-Frames, bevor ein Duckie gelöscht werden darf. |
| `obstacle_timeout` | Timeout für Hindernisdaten. |

---

## Lane-Borders und offene Seiten

| Parameter | Bedeutung |
|---|---|
| `lane_margin` | Sicherheitsabstand zu sichtbaren Linien. |
| `lane_timeout` | Timeout für Lane-Border-Daten. |
| `default_lane_left` | Fallback linke Grenze, wenn keine gültigen Daten vorhanden sind. |
| `default_lane_right` | Fallback rechte Grenze. |
| `open_side_width_bonus` | Bonus für freie Bereiche an einer offenen Seite, wenn eine Linie nicht sichtbar ist. |

---

## Ausweichregler

| Parameter | Bedeutung |
|---|---|
| `avoidance_vel` | Vorwärtsgeschwindigkeit während aktiver Ausweichfahrt. |
| `avoidance_steering_gain` | Verstärkung des Ausweichfehlers. |
| `avoidance_kp` | P-Anteil des Ausweich-PID. |
| `avoidance_ki` | I-Anteil des Ausweich-PID. |
| `avoidance_kd` | D-Anteil des Ausweich-PID. |
| `max_omega` | Maximale Winkelgeschwindigkeit, die der Controller ausgeben darf. |
| `min_vel` | Untere Grenze für Vorwärtsgeschwindigkeit, wenn ein Velocity-Override genutzt wird. |

---

## Ausweichseite und Zielglättung

| Parameter | Bedeutung |
|---|---|
| `avoidance_side_lock_time` | Zeit, in der die zuletzt gewählte Ausweichseite bevorzugt wird. |
| `avoidance_side_lock_bonus` | Score-Bonus für die zuletzt gewählte Ausweichseite. |
| `avoidance_target_smoothing_alpha` | Glättung des Ausweichzielpunkts. Höher = direktere Reaktion, niedriger = ruhiger. |
| `gap_width_bonus_weight` | Gewichtung der Intervallbreite bei der Auswahl des Ausweichbereichs. |
| `narrow_gap_behavior` | Verhalten, wenn kein ausreichend breiter Bereich gefunden wird. |

---

## Drehen auf der Stelle beim Ausweichen

| Parameter | Bedeutung |
|---|---|
| `avoidance_turn_in_place_error_enter` | Ab dieser Target-Abweichung dreht der Bot auf der Stelle statt vorwärts zu fahren. |
| `avoidance_turn_in_place_error_exit` | Unter dieser Abweichung verlässt der Bot den Turn-in-Place-Zustand. |
| `avoidance_turn_in_place_omega` | Winkelgeschwindigkeit beim Turn-in-Place-Ausweichen. |

---

## Post-Avoidance

| Parameter | Bedeutung |
|---|---|
| `avoidance_clear_hold_time` | Zeit, für die der letzte Ausweichzielpunkt nach Verschwinden des Duckies gehalten wird. |
| `lane_reentry_blend_time` | Dauer der weichen Rückführung vom Ausweichziel zum Lane-Target. |
| `reentry_vel` | Geschwindigkeit während der Rückführung in die Spur. |

---

## Blocked-Recovery

| Parameter | Bedeutung |
|---|---|
| `blocked_recovery_delay` | Wartezeit, bevor der Recovery-Scan startet. |
| `blocked_recovery_omega` | Winkelgeschwindigkeit während des Recovery-Scans. |
| `blocked_recovery_min_turn_time` | Mindestdauer des Recovery-Scans. |
| `blocked_recovery_min_angle_deg` | Mindest-Scanwinkel, aus dem eine Mindest-Scanzeit berechnet wird. |
| `blocked_recovery_max_angle_deg` | Maximaler Scanwinkel, aus dem eine maximale Scanzeit berechnet wird. |

---

## detect_obstacle_node.json

| Parameter | Bedeutung |
|---|---|
| `confidence_threshold` | Mindest-Confidence für YOLO-Detections. |
| `process_every_n_frames` | Nur jeder n-te Frame wird verarbeitet. Für geringe Latenz typischerweise `1`. |
| `input_size` | YOLO-Eingangsgröße. |
| `obstacle_region.x_min` | Linker Rand der Fallback-Obstacle-Region. |
| `obstacle_region.x_max` | Rechter Rand der Fallback-Obstacle-Region. |
| `obstacle_region.y_min` | Unterer/naher Bereich, ab dem Duckies als auf der Spur betrachtet werden. |
