# detect_lane_node.py

`detect_lane_node.py` erzeugt den normalen Lane-Error und die erkannten Fahrbahnbegrenzungen.

Der Node basiert auf einer Lane-Segmentierung des Kamerabildes. Für die Duckie-Challenge wird die Segmentierungsmaske zusätzlich mit den YOLO-Duckie-Boxen bereinigt, damit Duckies nicht als gelbe oder weiße Linie gewertet werden.

---

## Verarbeitungsschritte

```text
1. Kamerabild empfangen
2. unteren Bildbereich für Lane-Segmentierung ausschneiden
3. Segmentierungsmodell ausführen
4. Duckie-Bounding-Boxes aus der Maske löschen
5. gelbe und weiße Linienposition bestimmen
6. Lane-Center und Lane-Error berechnen
7. Debugbilder und Lane-Borders publizieren
```

---

## Ausschneiden der Duckie-Bounding-Boxes

Der Node subscribed zusätzlich auf:

```text
/<VEHICLE_NAME>/detect/duckie_BB
```

Die dort enthaltenen Bounding-Boxes werden in das Koordinatensystem des Lane-Crops umgerechnet. Danach werden alle Pixel innerhalb dieser Boxen auf Hintergrund gesetzt.

Dadurch werden Fehlsegmentierungen verhindert, bei denen zum Beispiel eine gelbe Ente als gelbe Linie erkannt wird.

Wichtig: Es werden nicht nur gelbe Pixel gelöscht. Innerhalb der Box wird die komplette Segmentierungsmaske bereinigt. Dadurch werden auch weiße Fehlsegmente in Duckie-Bereichen entfernt.

---

## Lane-Borders

Der Node publiziert:

```text
/<VEHICLE_NAME>/detect/lane_borders
```

Beispiel:

```json
{
  "yellow_x": 0.18,
  "white_x": 0.82,
  "lane_center_x": 0.50,
  "yellow_valid": true,
  "white_valid": true,
  "valid": true
}
```

Die Werte sind normiert:

```text
0.0 = linker Bildrand
1.0 = rechter Bildrand
```

`yellow_valid` und `white_valid` sind für den Controller wichtig. Sie sagen, ob die jeweilige Linie wirklich erkannt wurde oder nur ein Fallback-Wert vorliegt.

---

## Lane-Error

Der Lane-Error wird publiziert auf:

```text
/<VEHICLE_NAME>/detect/lane
```

Er entspricht ungefähr:

```text
error = 1 - 2 * lane_center_x
```

Damit gilt:

```text
error > 0   Ziel liegt links
error < 0   Ziel liegt rechts
error = 0   Ziel liegt mittig
```

Dieser Wert ist die Basis für das normale Lane-Following im `control_lane_node.py`.

---

## Debugbilder

Der Node publiziert Debugbilder für das Dashboard:

```text
/<VEHICLE_NAME>/debug/lane_croped
/<VEHICLE_NAME>/debug/lane_white
/<VEHICLE_NAME>/debug/lane_yellow
```

Diese Bilder helfen zu prüfen, ob die Segmentierung und das Mask-Cleanup korrekt funktionieren.
