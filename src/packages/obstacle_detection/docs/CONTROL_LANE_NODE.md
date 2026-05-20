# control_lane_node.py

`control_lane_node.py` ist der zentrale Fahr-Node dieses Packages. Er verbindet das normale Lane-Following aus Challenge 1 mit einer lokalen Ausweichlogik für Duckies.

Der Node fährt grundsätzlich **nicht dauerhaft nach Hindernisplanung**, sondern nutzt das Lane-Following als Basis. Die Ausweichlogik greift nur ein, wenn ein ausreichend großes Duckie im relevanten Bildbereich liegt und den normalen Lane-Target-Bereich blockiert.

---

## Eingänge und Ausgänge

### Eingänge

```text
/<VEHICLE_NAME>/detect/lane
```

Normaler Lane-Error als `Float64`. Dieser Wert treibt das Challenge-1-Lane-Following.

```text
/<VEHICLE_NAME>/detect/lane_borders
```

JSON mit gelber und weißer Linienposition sowie Validitätsinformationen.

```text
/<VEHICLE_NAME>/detect/duckie_BB
```

JSON mit allen erkannten Duckie-Bounding-Boxes.

### Ausgänge

```text
/<VEHICLE_NAME>/car_cmd_switch_node/cmd
```

Fahrbefehl als `Twist2DStamped` mit `v` und `omega`.

```text
/<VEHICLE_NAME>/debug/free_path_plan
```

JSON-Debugausgabe für Dashboard und Analyse.

---

## Gesamtlogik

Die Fahrentscheidung passiert bei jeder neuen Lane-Error-Nachricht in `cbFollowLane()`.

Vereinfacht:

```text
1. Lane-Error empfangen
2. choose_avoidance_target() aufrufen
3. Wenn kein Duckie relevant ist:
       normales Lane-Following
4. Wenn Duckie blockiert:
       Ausweichziel berechnen
5. Wenn kein gültiger Bereich existiert:
       Blocked-Recovery
6. Wenn Ausweichziel sehr weit weg liegt:
       zuerst auf der Stelle in Richtung Ziel drehen
7. PID-Regelung berechnen
8. v und omega publizieren
```

---

## 1. Normales Lane-Following

Wenn kein relevanter Duckie-Bereich vorhanden ist, wird der Lane-Error direkt geregelt:

```text
/detect/lane -> PID -> v, omega
```

Dieser Fall ist bewusst nahe an Challenge 1 gehalten. Dadurch bleibt das normale Fahrverhalten außerhalb von Duckies möglichst stabil und bekannt.

Der normale Regler nutzt die Parameter aus `pid`:

```text
p, i, d, max_vel
```

---

## 2. Fahrbereich aus Lane-Borders

Die Straße wird über die erkannten Linien begrenzt:

```text
gelbe Linie = linke harte Grenze
weiße Linie = rechte harte Grenze
```

Dabei wird nicht blind jeder Fallback-Wert verwendet. Der Lane-Node publiziert zusätzlich:

```json
{
  "yellow_valid": true,
  "white_valid": true
}
```

Dadurch kann der Controller unterscheiden:

```text
Linie wirklich erkannt
oder nur Fallback-Wert vorhanden
```

### Wenn beide Linien sichtbar sind

```text
Fahrbereich = gelbe Linie bis weiße Linie
```

### Wenn nur die weiße Linie sichtbar ist

```text
rechte Grenze = weiße Linie
linke Seite = offen
```

Das ist wichtig, weil die Challenge-Spur breiter sein kann als das sichtbare Kamerabild. Wenn die gelbe Linie nicht sichtbar ist, darf der linke Bereich nicht künstlich blockiert werden.

### Wenn nur die gelbe Linie sichtbar ist

```text
linke Grenze = gelbe Linie
rechte Seite = offen
```

### Wenn keine aktuelle Linie vorhanden ist

Der Controller nutzt kurzzeitig den letzten gültigen Fahrbereich oder die Default-Grenzen.

---

## 3. Duckie-Filterung

Der Controller übernimmt nicht jede YOLO-Detection sofort als relevantes Hindernis.

Ein Duckie wird nur aktiv berücksichtigt, wenn es:

```text
- Klasse duckie hat,
- groß genug ist,
- im vertikalen Planungsfenster liegt,
- den normalen Fahrweg beeinflusst.
```

### Mindestgröße

Über diese Parameter werden weit entfernte oder sehr kleine Duckies ignoriert:

```text
min_duckie_width_px
min_duckie_height_px
min_duckie_area_px
```

Dadurch reagiert der Bot nicht zu früh auf kleine Detections am Horizont.

### Detection-Hold

YOLO kann einzelne Frames verpassen. Damit der rote Sperrbereich nicht flackert, hält der Controller erkannte Duckies kurz weiter:

```text
duckie_hold_time
duckie_missed_frames_before_clear
```

Das macht die Planung reproduzierbarer.

---

## 4. Blockierte und freie Bereiche

Für jedes relevante Duckie wird aus der Bounding-Box ein horizontaler Sperrbereich berechnet:

```text
blocked_left  = duckie_xmin - duckie_x_margin
blocked_right = duckie_xmax + duckie_x_margin
```

Der Sperrbereich wird auf den aktuellen Fahrbereich begrenzt. Mehrere überlappende Sperrbereiche werden zusammengeführt.

Aus dem Fahrbereich und den blockierten Intervallen entstehen freie Intervalle:

```text
Fahrbereich: [lane_left, lane_right]
Blockiert:  [b1_left, b1_right], [b2_left, b2_right]
Frei:       Bereiche dazwischen
```

Die Mindestbreite eines gültigen freien Bereichs wird über `min_free_width_px` festgelegt.

---

## 5. Offene Seite bei fehlender Linie

Wenn eine Linie fehlt, wird diese Seite als offen behandelt.

Beispiel:

```text
weiß sichtbar, gelb nicht sichtbar
-> rechts harte Grenze
-> links offene Seite
```

Damit der offene Bereich in der Bewertung nicht fälschlich als zu klein gilt, bekommt ein freies Intervall am offenen Bildrand einen Bonus:

```text
open_side_width_bonus
```

Das bedeutet nicht, dass der Bot Linien überfahren darf. Sichtbare Linien bleiben harte Grenzen. Nur die nicht sichtbare Seite wird als offen interpretiert.

---

## 6. Auswahl der Ausweichrichtung

Der Controller fährt nicht grundsätzlich zur breitesten Lücke. Die Entscheidung ist gestuft:

```text
1. Normalen Lane-Target aus Lane-Error berechnen.
2. Prüfen, ob dieser Target-Bereich von einem Duckie blockiert ist.
3. Wenn nein:
       weiter Lane-Following.
4. Wenn ja:
       freien Bereich links oder rechts neben dem Duckie suchen.
5. Zielpunkt innerhalb des ausgewählten freien Bereichs setzen.
```

Der Abstand zum Duckie wird durch `duckie_x_margin`, `lane_target_block_margin` und `escape_clearance` bestimmt.

### Side-Lock

Damit der Bot nicht zwischen links und rechts hin und her wechselt, wird die gewählte Ausweichseite kurz bevorzugt:

```text
avoidance_side_lock_time
avoidance_side_lock_bonus
```

---

## 7. Drehen auf der Stelle bei großem Ausweichfehler

Wenn das Ausweichziel weit links oder rechts liegt, reicht normales Vorwärtsfahren mit Lenkung manchmal nicht aus. Der Bot würde sonst während des Drehens weiter auf die Ente zufahren.

Dafür gibt es einen zusätzlichen Zustand:

```text
avoidance_turn_in_place
```

Ablauf:

```text
1. Ausweichziel liegt weit außerhalb der Bildmitte.
2. Vorwärtsgeschwindigkeit wird auf 0 gesetzt.
3. Bot dreht auf der Stelle in Richtung target_x.
4. Sobald der Fehler klein genug ist, fährt er wieder mit avoidance_vel weiter.
```

Die Richtung wird aus dem Vorzeichen des Ausweichfehlers bestimmt:

```text
target links  -> links drehen
target rechts -> rechts drehen
```

Wichtige Parameter:

```text
avoidance_turn_in_place_error_enter
avoidance_turn_in_place_error_exit
avoidance_turn_in_place_omega
```

Die unterschiedlichen Enter-/Exit-Schwellen verhindern ständiges Umschalten.

---

## 8. Blocked-Recovery-Zustand

Manchmal gibt es keinen gültigen freien Bereich, zum Beispiel in engen Kurven oder wenn die sichtbare Szene ungünstig ist.

Dann liefert `choose_avoidance_target()` keinen gültigen Zielpunkt. In diesem Fall greift der Blocked-Recovery-Zustand.

Ablauf:

```text
1. Kein gültiger freier Bereich gefunden.
2. Bot wartet zunächst blocked_recovery_delay Sekunden.
3. Danach dreht er langsam und scannt die Szene.
4. Während des Scans merkt er sich den besten gefundenen Zielbereich.
5. Nach der Scanphase nutzt er diesen Bereich oder dreht zur besten Position zurück.
```

Wichtige Parameter:

```text
blocked_recovery_delay
blocked_recovery_omega
blocked_recovery_min_turn_time
blocked_recovery_min_angle_deg
blocked_recovery_max_angle_deg
```

Dieser Zustand ist nur für Situationen gedacht, in denen keine direkte Durchfahrt berechnet werden kann.

---

## 9. Post-Avoidance und Rückkehr zur Spur

Wenn das Duckie aus dem Kamerabild verschwindet, ist der Bot oft noch nicht vollständig daran vorbei. Würde er sofort wieder zur Spurmitte regeln, könnte er zurück in die Ente fahren.

Deshalb gibt es zwei Phasen:

```text
1. avoidance_clear_hold_time:
       letzter Ausweichzielpunkt wird kurz gehalten

2. lane_reentry_blend_time:
       Zielpunkt wird weich zurück zum normalen Lane-Target überblendet
```

Die Geschwindigkeit in dieser Phase wird über `reentry_vel` bestimmt.

---

## 10. PID-Regelung

Es gibt zwei PID-Sätze:

### Normaler Lane-PID

```text
p, i, d
```

Wird im normalen Lane-Following verwendet.

### Avoidance-PID

```text
avoidance_kp
avoidance_ki
avoidance_kd
```

Wird beim aktiven Ausweichen verwendet. Der Moduswechsel setzt den Integralanteil zurück und unterdrückt den D-Sprung beim Umschalten.

Zusätzlich wird der Ausweichfehler mit `avoidance_steering_gain` skaliert.

---

## 11. Debug-Ausgabe

Der Controller publiziert kontinuierlich ein JSON auf:

```text
/<VEHICLE_NAME>/debug/free_path_plan
```

Typische Felder:

```text
reason                    aktueller Zustand / Entscheidungsgrund
avoidance_active           ob Ausweichlogik aktiv ist
target_x                   aktueller Zielpunkt
lane_target_x              normaler Zielpunkt aus Lane-Following
blocked_intervals          rote Sperrbereiche
free_intervals             freie Bereiche
selected_free_interval     gewählter Ausweichbereich
left_open / right_open     ob eine Seite wegen fehlender Linie offen ist
v / omega                  aktuell berechneter Fahrbefehl
```

Diese Daten werden vom Dashboard visualisiert und sind die wichtigste Quelle für Debugging.
