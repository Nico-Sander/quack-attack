# ch3_obstacle_avoidance

ROS-Package für die Duckiebot-Challenge **„Enten auf der Straße umfahren“**.

Das Package kombiniert drei Aufgaben:

1. Fahrstreifen erkennen.
2. Duckies mit YOLO erkennen.
3. Während der Fahrt entscheiden, ob normales Lane-Following ausreicht oder ob ein Ausweichmanöver nötig ist.

Die zentrale Idee ist bewusst einfach gehalten:

```text
Normalfall (CRUISE):
    /detect/lane -> P-Lenkung -> car_cmd

Duckie blockiert den Fahrweg (AVOID):
    Duckie-Bounding-Boxen + Spurgrenzen -> freie Lücken
    -> Ziel in die beste Lücke, langsamer vorbei

Kein Weg nach vorn (ESCAPE_ROTATE):
    auf der Stelle drehen, bis sich eine Lücke zeigt (nie anhalten)
```

Der wichtigste Node ist `control_lane_node.py`. Er nutzt Lane-Following als Route und weicht Duckies mit einem „follow-the-gap"-Regler aus, der konstruktionsbedingt nie einfriert.

---

## Package-Struktur

```text
ch3_obstacle_avoidance/
├── config/
│   ├── control_lane_node.json        # 14 Parameter für Regler, Ausweichen und Recovery
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

Dieser Node entscheidet, wie der Duckiebot fährt. Er ist als „follow-the-gap"-Regler mit
einer kleinen Zustandsmaschine (CRUISE / AVOID / ESCAPE_ROTATE) aufgebaut. Leitprinzip:
**genau eine Stelle** schreibt den Fahrbefehl und erzwingt die Invariante „nie `v==0 UND
omega==0`". Ohne Weg nach vorn dreht sich der Bot, um eine Lücke zu suchen, statt anzuhalten.

Kurzüberblick:

1. **Korridor** aus den (entprellten) Linienpositionen; sichtbare Linien sind harte Grenzen,
   der Zielpunkt wird immer hineingeklemmt.
2. **Blockierte/freie Intervalle** aus den nähe-abhängig verbreiterten Duckie-Boxen; die
   breiteste freie Lücke wird gewählt.
3. **Zustandsmaschine**: freie Fahrbahn → `CRUISE` (Lane-Following); Ente im Weg → `AVOID`
   (langsamer, um sie herumlenken); kein Weg nach vorn → `ESCAPE_ROTATE` (auf der Stelle
   drehen, bis sich eine Lücke zeigt, mit schrittweiser Lockerung als Anti-Freeze-Garantie).

Die Entscheidungslogik (`GapPlanner`) ist `rospy`-frei und ohne ROS testbar.

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
rosrun ch3_obstacle_avoidance dashboard_node.py
rosrun ch3_obstacle_avoidance detect_lane_node.py
rosrun ch3_obstacle_avoidance detect_obstacle_node.py
rosrun ch3_obstacle_avoidance control_lane_node.py

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
