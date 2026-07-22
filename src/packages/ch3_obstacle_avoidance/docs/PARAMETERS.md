# Parameterübersicht

Die Parameter liegen in:

```text
config/control_lane_node.json
config/detect_obstacle_node.json
```

Die Werte in `control_lane_node.json` sind unter `parameters.controller` gruppiert. Der
Controller wurde bewusst auf wenige, gut interpretierbare Parameter reduziert (15 Stück).
Geometrie- und Debounce-Konstanten (Planungsband, Frame-Hysterese, Bildgrößen) sind als
Konstanten direkt im Node hinterlegt und nicht als Parameter ausgelegt.

Wertebereiche der Hardware zur Orientierung:

```text
v:     0.2 = schnell, 0.1 = mittel, 0.05 = Reibungsgrenze (Räder bleiben stehen)
omega: 1   = langsame Drehung, 4 = sehr abrupte Drehung
```

---

## Geschwindigkeiten

| Parameter | Default | Bedeutung |
|---|---|---|
| `v_cruise` | 0.10 | Vorwärtsgeschwindigkeit bei freier Fahrbahn (normales Lane-Following). |
| `v_avoid` | 0.06 | Vorwärtsgeschwindigkeit beim Ausweichen um ein Duckie (oberes Ende der Nähe-Rampe). |
| `v_min` | 0.06 | Kriechgeschwindigkeit mit kleinem Abstand über der Reibungsgrenze (~0.05). Wird bei naher Ente oder unbekannter Geometrie genutzt. Der Bot fährt nie langsamer, sondern dreht sich stattdessen (siehe `ESCAPE_ROTATE`). |

---

## Lenkung

| Parameter | Default | Bedeutung |
|---|---|---|
| `k_steer` | 6.0 | Proportionaler Lenkfaktor. `omega = k_steer * (0.5 - target_x) * 2`, was im Lane-Following `k_steer * lane_error` entspricht — dies **ist** der Lane-P-Gain. Entspricht dem bewährten Challenge-1-Wert. Erhöhen, wenn der Bot zu träge lenkt und Linien überfährt; senken bei Oszillation. |
| `omega_rotate` | 3.0 | Drehrate beim Drehen auf der Stelle im `ESCAPE_ROTATE`-Zustand. Muss hoch genug sein, um bei `v=0` die Haftreibung zu überwinden — zu niedrig, und der Bot summt nur, dreht sich aber nicht. |
| `omega_max` | 4.0 | Harte Obergrenze für `|omega|` jeder Ausgabe. 4 = das „sehr abrupte" Ende des nutzbaren Bereichs. |

---

## Grenzen und Sicherheitsabstände

| Parameter | Default | Bedeutung |
|---|---|---|
| `lane_margin` | 0.08 | Harter Abstand zu jeder erkannten Linie. Der befahrbare Korridor ist `[linke_Linie + margin, rechte_Linie - margin]`. Der Lenk-Zielpunkt wird immer in diesen Korridor geklemmt, damit keine Linie überfahren wird. |
| `duckie_margin_base` | 0.06 | Seitliche Verbreiterung, die jeder Seite einer weit entfernten Duckie-Box hinzugefügt wird. |
| `duckie_margin_gain` | 0.12 | Zusätzliche Verbreiterung proportional zur Nähe (`ymax`). Eine nahe Ente wird um `base + gain` verbreitert. |
| `gap_min_width` | 0.16 | Mindestbreite (normiert) einer freien Lücke, damit sie als befahrbar gilt. Darunter dreht sich der Bot statt zu fahren. |

---

## Nähe-Reaktion

Diese drei Schwellen arbeiten mit der Nähe einer Ente (`ymax`, unten im Bild = näher) und
sollten geordnet bleiben: `react_ymax < front_slow_ymax < front_block_ymax`.

| Parameter | Default | Bedeutung |
|---|---|---|
| `react_ymax` | 0.62 | Nähe, die eine Ente erreichen muss, damit sie überhaupt zählt — erst dann erzeugt sie ihr blockiertes Intervall (rote Box) und kann ein Ausweichen auslösen. **HÖHER = reagiert näher/später, NIEDRIGER = reagiert weiter/früher.** Dies ist der Regler gegen „reagiert zu früh". |
| `front_slow_ymax` | 0.72 | `ymax` der nächsten Ente direkt voraus, ab dem die Geschwindigkeit Richtung `v_min` heruntergeregelt wird. Über `react_ymax` halten. |
| `front_block_ymax` | 0.88 | `ymax`, ab dem die Front als blockiert gilt (kein Weg nach vorn) → Wechsel in `ESCAPE_ROTATE`. |

---

## Recovery / Anti-Freeze

| Parameter | Default | Bedeutung |
|---|---|---|
| `escape_min_dwell` | 0.25 | Mindestzeit im `ESCAPE_ROTATE`, bevor er wieder verlassen werden darf (verhindert Zittern zwischen Drehen und Fahren). **Wichtig:** `escape_min_dwell * omega_rotate` ist der Mindest-Drehwinkel, bevor der Zustand neu bewertet werden darf. Bei 0.25 × 1.5 sind das ~21°. Deutlich größere Werte drehen die Fahrbahnlinien aus dem Kamerabild, bevor der Zustand überhaupt neu geprüft wird — genau so fährt der Bot blind über die Markierung. |
| `escape_relax_after` | 2.0 | Nach dieser Drehzeit ohne befahrbare Lücke werden `gap_min_width` und die Duckie-Verbreiterung schrittweise verkleinert, bis sich eine Lücke öffnet (Garantie gegen dauerhaftes Feststecken). **Achtung:** In einer echten Sackgasse heißt das, dass die Sicherheitsabstände so lange schrumpfen, bis eine zu enge Lücke als befahrbar gilt. Ein echtes Rückwärts-Manöver (`ESCAPE_REVERSE`) fehlt noch. |
| `front_slice_half` | 0.04 | Halbe Breite des Front-Streifens („ist etwas in meinem Weg?") **um die tatsächlich angesteuerte Spalte**. Entspricht der **Roboterbreite**, nicht einer Komfortzone. **Harte Bedingung: kleiner als `gap_min_width/2`.** Ist er breiter, liegen genau die beiden Enten, die die Lücke bilden, im Streifen, während der Bot zwischen ihnen hindurchfährt — `front_block` löst an den eigenen Lückenrändern aus und das Manöver bricht nach ein bis zwei Frames ab. Der Node warnt beim Start, wenn die Bedingung verletzt ist. |
| `avoid_min_dwell` | 0.8 | Sekunden, die `AVOID` gehalten wird, nachdem es auf einer befahrbaren Lücke gestartet ist — auch wenn die Lücke kurzzeitig zu schmal misst. Das ist der Knopf für „fahr da beherzt durch". Die Duckie-Verbreiterung wächst mit der Nähe, eine auf Distanz akzeptierte Lücke wird beim Heranfahren also **zwangsläufig** schmaler; ohne Dwell bricht das Manöver konstruktionsbedingt auf halbem Weg ab. Eine wirklich nahe Ente (`front_block_ymax`) bricht weiterhin sofort ab. |
| `lane_hold_frames` | 12 | Aufeinanderfolgende ungültige Lane-Border-Messungen, bevor diese Seite als offen gilt und der Korridor sich dort öffnet. Das ist das **Linien-Gedächtnis**: solange die Serie darunter liegt, bleibt die zuletzt bekannte Linienposition eine harte Grenze. Zu klein → ein kurzer Sichtverlust (z. B. während einer Drehung) hebt die Grenze auf und der Bot fährt über die Markierung. Einheit sind **Frames, nicht Sekunden** — die Wanduhr-Zeit hängt von der Publish-Rate von `detect_lane_node` ab (`rostopic hz /$VEHICLE_NAME/detect/lane_borders`). |
| `duckie_hold_time` | 1.0 | Sekunden, für die eine Duckie-Box nach Detektionsausfall gehalten wird (Blindflug). Höhere Werte verhindern, dass eine Drehung die Ente vergisst, die den Escape ausgelöst hat. Aber: die Box wird in **Bild-x** gespeichert, sitzt nach einer Drehung also am falschen Ort. Über ~1.5 sind Phantom-Hindernisse zu erwarten. |

---

## Fest verdrahtete Konstanten (nicht als Parameter)

Diese sind aus der Kamerageometrie bzw. der Bildrate abgeleitet und stehen oben im Node:

```text
PLAN_Y_MAX = 1.0       # unterer Rand des Planungsbands (oberer Rand = react_ymax)
LANE_TIMEOUT = 1.0     # s, bevor Lane-Border-Daten als veraltet gelten
MIN_ESCAPE_OMEGA = 0.8 # untere Schranke für omega_rotate (Haftreibung), siehe unten
GAP_STICKY_BONUS = 0.06 # Breiten-Bonus für die Lücke, in die schon gefahren wird
HYSTERESIS_MARGIN = 0.10 # Schwelle gegen Links/Rechts-Flackern
GAP_INSET = 0.04       # Abstand des Zielpunkts zu den Rändern der gewählten Lücke
CLEAR_HOLD_TIME = 0.6  # s freie Fahrbahn, bevor AVOID zurück zu CRUISE fällt
MIN_DUCKIE_WIDTH/HEIGHT = 0.04  # YOLO-Rauschfilter
```

`omega_rotate` wird im Konstruktor auf `MIN_ESCAPE_OMEGA` nach unten begrenzt. Damit
benutzt der Never-Freeze-Guard direkt `omega_rotate`, statt wie früher eine eigene
fest verdrahtete Untergrenze (2.0) zu führen, die einem bewusst niedriger
eingestellten `omega_rotate` widersprochen hat.

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
