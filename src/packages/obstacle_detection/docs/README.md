# obstacle_detection

ROS-Package für die Duckiebot-Challenge **„Enten auf der Straße umfahren“**.

Das Package kombiniert drei Aufgaben:

1. Fahrstreifen erkennen.
2. Duckies mit YOLO erkennen.
3. Während der Fahrt entscheiden, ob normales Lane-Following ausreicht oder ob ein Ausweichmanöver nötig ist.

Die zentrale Idee ist bewusst einfach gehalten:

```text
Normalfall:
    /detect/lane -> PID-Regler -> car_cmd

Duckie blockiert den normalen Fahrweg:
    Duckie-Bounding-Boxen + Spurgrenzen -> freie Fahrbereiche
    -> sicheres Ziel links/rechts neben dem Duckie
    -> langsamer und kontrollierter ausweichen
```

Der wichtigste Node ist `control_lane_node.py`. Er übernimmt das bewährte Lane-Following aus Challenge 1 als Basis und aktiviert die Ausweichlogik nur bei relevanten Duckies.

---

## Package-Struktur

```text
obstacle_detection/
├── config/
│   ├── control_lane_node.json        # Parameter für Regler, Ausweichlogik und Recovery
│   └── detect_obstacle_node.json     # Parameter für YOLO-Duckie-Erkennung
├── docs/
│   ├── CONTROL_LANE_NODE.md          # Detailbeschreibung der Fahrlogik
│   ├── DETECT_LANE_NODE.md           # Segmentierung, Mask-Cleanup und Lane-Borders
│   ├── PARAMETERS.md                 # Erklärung aller Parameter
│   ├── TOPICS.md                     # ROS-Topic-Übersicht
│   └── DEBUGGING.md                  # Hinweise für Tests und typische Fehlerbilder
├── models/
│   ├── YOLOv11_duckie_detection_modell.pt
│   ├── lane_segmentation.pth
│   └── lane_segmentation_002_model.pth
└── src/
    ├── control_lane_node.py          # Lane-Following + Duckie-Ausweichlogik
    ├── detect_lane_node.py           # Lane-Segmentierung und Lane-Error
    ├── detect_obstacle_node.py       # YOLO-Duckie-Erkennung
    └── dashboard_node.py             # Debug-Dashboard
```

---

## Nodes im Überblick

### `detect_lane_node.py`

Dieser Node segmentiert das Kamerabild in Fahrbahnmarkierungen und erzeugt daraus den normalen Lane-Error.

Er publiziert unter anderem:

```text
/<VEHICLE_NAME>/detect/lane
/<VEHICLE_NAME>/detect/lane_borders
/<VEHICLE_NAME>/debug/lane_croped
/<VEHICLE_NAME>/debug/lane_white
/<VEHICLE_NAME>/debug/lane_yellow
```

Für diese Challenge wurde zusätzlich eine wichtige Schutzmaßnahme eingebaut: erkannte Duckie-Bounding-Boxes werden aus der Segmentierungsmaske entfernt, bevor gelbe und weiße Linienpositionen berechnet werden. Dadurch kann eine gelbe oder weiße Duckie-Fläche die Lane-Detection nicht so leicht verfälschen.

Details: [`docs/DETECT_LANE_NODE.md`](docs/DETECT_LANE_NODE.md)

---

### `detect_obstacle_node.py`

Dieser Node nutzt das trainierte YOLO-Modell zur Erkennung von Duckies. Er verändert nicht die Fahrlogik, sondern liefert nur strukturierte Hindernisinformationen.

Wichtig ist vor allem:

```text
/<VEHICLE_NAME>/detect/duckie_BB
```

Dieses Topic enthält eine JSON-Liste aller erkannten Duckies mit normierten Bounding-Boxes.

Details: [`docs/TOPICS.md`](docs/TOPICS.md)

---

### `control_lane_node.py`

Dieser Node entscheidet, wie der Duckiebot fährt.

Die Logik besteht aus mehreren Blöcken:

1. **Normales Lane-Following**  
   Wenn kein relevantes Duckie den Fahrweg blockiert, wird direkt der Lane-Error aus `/detect/lane` geregelt.

2. **Fahrbereich bestimmen**  
   Sichtbare gelbe und weiße Linien werden als harte Grenzen verwendet. Wenn eine Linie nicht sichtbar ist, wird diese Seite als offen behandelt.

3. **Duckies filtern**  
   Sehr kleine oder zu weit entfernte Duckies werden ignoriert. Duckies werden außerdem für kurze Zeit gehalten, damit einzelne YOLO-Aussetzer nicht sofort zu flackernden Sperrbereichen führen.

4. **Sperrbereiche und freie Bereiche berechnen**  
   Die Bounding-Boxes relevanter Duckies werden mit Sicherheitsmargen horizontal als blockierte Intervalle in den Fahrbereich gelegt. Daraus entstehen freie Intervalle.

5. **Ausweichziel wählen**  
   Der Node fährt nicht blind zur breitesten Lücke. Er prüft zuerst, ob der normale Lane-Target-Bereich blockiert ist. Nur dann wird ein Ausweichziel links oder rechts neben dem Duckie gewählt.

6. **Starkes Ausweichen auf der Stelle**  
   Wenn das Ausweichziel sehr weit links oder rechts liegt, fährt der Bot nicht sofort vorwärts. Er dreht zuerst auf der Stelle in Richtung Ziel, bis der Fehler kleiner wird.

7. **Post-Avoidance**  
   Wenn das Duckie aus dem Sichtfeld verschwindet, wird nicht abrupt zurück zur Spurmitte geregelt. Der letzte Ausweichkurs wird kurz gehalten und dann sanft wieder in Lane-Following überführt.

8. **Blocked-Recovery**  
   Wenn kein gültiger Fahrbereich gefunden wird, wartet der Bot kurz und scannt dann langsam durch Drehen. Dabei merkt sich der Node den besten gefundenen Zielbereich und fährt danach weiter.

Details: [`docs/CONTROL_LANE_NODE.md`](docs/CONTROL_LANE_NODE.md)

---

### `dashboard_node.py`

Das Dashboard zeigt die wichtigsten Debugbilder:

- Lane-Bild mit erkannten Linien.
- Weiße Maske.
- Gelbe Maske.
- Obstacle-/Planning-Overlay mit blockierten und freien Bereichen.

Farben im Planning-Overlay:

```text
Rot    = durch Duckie gesperrter Bereich
Grün   = freie Bereiche
Gelb   = ausgewählter Fahrbereich
Blau   = aktueller Zielpunkt target_x
```

---

## Starten

Im Docker-Container:

```bash
cd /workspace
catkin_make
source devel/setup.bash
```

Dann in separaten Terminals:

```bash
rosrun obstacle_detection dashboard_node.py
rosrun obstacle_detection detect_lane_node.py
rosrun obstacle_detection detect_obstacle_node.py
rosrun obstacle_detection control_lane_node.py

```

Wichtig: `VEHICLE_NAME` muss zum Roboternamen passen, zum Beispiel:

```bash
export VEHICLE_NAME=track
```


## Wichtige Dokumente

- [`docs/CONTROL_LANE_NODE.md`](docs/CONTROL_LANE_NODE.md): vollständige Beschreibung der Fahrlogik.
- [`docs/PARAMETERS.md`](docs/PARAMETERS.md): Erklärung aller Parameter.
- [`docs/TOPICS.md`](docs/TOPICS.md): Eingangs-, Ausgangs- und Debug-Topics.
- [`docs/DEBUGGING.md`](docs/DEBUGGING.md): typische Probleme und wo man sie prüft.
