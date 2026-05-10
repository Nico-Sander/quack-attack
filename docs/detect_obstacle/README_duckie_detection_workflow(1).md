# Debug- und Test-Workflow für Duckie Detection

Diese Anleitung beschreibt, welche Terminals/Nodes gestartet werden müssen, um die Duckie-Erkennung, das Debug-Dashboard und die ROS-Topics zu testen.

## Wichtig: Nach Codeänderungen

Wenn nur Python-Code, JSON-Config oder Modell-Dateien geändert wurden, reicht meistens:

```bash
source devel/setup.bash
```

Wenn ein neues Package angelegt wurde oder `package.xml` / `CMakeLists.txt` geändert wurden, dann im Container einmal ausführen:

```bash
cd /workspace
catkin_make
source devel/setup.bash
```

---

## Terminal 0: Container starten und Display freigeben

Auf dem Host-PC, nicht im Container:

```bash
cd ~/repos/quack-attack
xhost +local:root
docker compose up -d
```

In den Container springen:

```bash
docker exec -it duckie_ros bash
```

---

## Terminal 1: ROS Master starten

Dieses Terminal offen lassen:

```bash
docker exec -it duckie_ros bash
cd /workspace
source devel/setup.bash
roscore
```

Zweck: Startet den zentralen ROS-Master. Ohne `roscore` können die Nodes nicht miteinander kommunizieren.

---

## Terminal 2: Duckie Detection Node starten

```bash
docker exec -it duckie_ros bash
cd /workspace
source devel/setup.bash
rosrun obstacle_detection detect_obstacle_node.py
```

Zweck: Abonniert das Kamerabild, führt die YOLO-Duckie-Erkennung aus und published:

```text
/trick/detect/duckie
/trick/detect/obstacle
/trick/debug/obstacle_detection
```

Erwartete Logs:

```text
YOLOv11 model loaded successfully
subscribing to /trick/camera_node/image/compressed
```

---

## Terminal 3: Dashboard starten

Vorher sicherstellen, dass auf dem Host einmal ausgeführt wurde:

```bash
xhost +local:root
```

Dann im Container:

```bash
docker exec -it duckie_ros bash
cd /workspace
source devel/setup.bash
rosrun follow_lane dashboard_node.py
```

Zweck: Zeigt ein OpenCV-Dashboard mit Debug-Bildern, unter anderem:

```text
Annotated Lane
White Mask
Yellow Mask
Red Mask
Obstacle Detection
```

Im Bereich `Obstacle Detection` sollte das Kamerabild mit Suchbereich und Bounding Boxes sichtbar sein.

---

## Terminal 4: Detection-Ausgabe prüfen

```bash
docker exec -it duckie_ros bash
cd /workspace
source devel/setup.bash
rostopic echo /trick/detect/duckie
```

Zweck: Zeigt das einfache Detection-Signal:

```text
data: 0.0  -> keine Ente auf der Spur erkannt
data: 1.0  -> Ente auf der Spur erkannt
```

Optional strukturierte Ausgabe prüfen:

```bash
rostopic echo /trick/detect/obstacle
```

Dort erscheinen JSON-Informationen wie Klasse, Confidence, Bounding Box und `obstacle_on_lane`.

---

## Optional: Prüfen, ob Kamerabilder ankommen

```bash
docker exec -it duckie_ros bash
cd /workspace
source devel/setup.bash
rostopic hz /trick/camera_node/image/compressed
```

Zweck: Prüft, ob die Kamera Bilder published. Wenn hier keine Frequenz erscheint, bekommt `detect_obstacle_node.py` keine Bilder.

---

## Optional: Prüfen, ob Debug-Bild published wird

```bash
docker exec -it duckie_ros bash
cd /workspace
source devel/setup.bash
rostopic hz /trick/debug/obstacle_detection
```

Zweck: Prüft, ob der Detection-Node das Debug-Bild mit Bounding Boxes published.

---

## Optional: Fake-Kamera ohne Duckiebot starten

Falls keine echte Kamera verfügbar ist:

```bash
docker exec -it duckie_ros bash
cd /workspace
source devel/setup.bash
python3 - <<'PY'
import cv2
import rospy
import numpy as np
from sensor_msgs.msg import CompressedImage

topic = "/trick/camera_node/image/compressed"
rospy.init_node("fake_camera_publisher")
pub = rospy.Publisher(topic, CompressedImage, queue_size=1)

img = np.zeros((480, 640, 3), dtype=np.uint8)
cv2.putText(
    img,
    "fake camera frame",
    (80, 240),
    cv2.FONT_HERSHEY_SIMPLEX,
    1.0,
    (255, 255, 255),
    2,
)

rate = rospy.Rate(5)
print(f"Publishing fake frames to {topic}")

while not rospy.is_shutdown():
    msg = CompressedImage()
    msg.header.stamp = rospy.Time.now()
    msg.format = "jpeg"
    msg.data = np.array(cv2.imencode(".jpg", img)[1]).tobytes()
    pub.publish(msg)
    rate.sleep()
PY
```

Zweck: Testet, ob der Detection-Node und das Dashboard grundsätzlich funktionieren, auch ohne echte Kamera.

---

## Typischer Startablauf

### 1. Host-Terminal

```bash
cd ~/repos/quack-attack
xhost +local:root
docker compose up -d
```

### 2. Terminal 1

```bash
docker exec -it duckie_ros bash
cd /workspace
source devel/setup.bash
roscore
```

### 3. Terminal 2

```bash
docker exec -it duckie_ros bash
cd /workspace
source devel/setup.bash
rosrun obstacle_detection detect_obstacle_node.py
```

### 4. Terminal 3

```bash
docker exec -it duckie_ros bash
cd /workspace
source devel/setup.bash
rosrun follow_lane dashboard_node.py
```

### 5. Terminal 4

```bash
docker exec -it duckie_ros bash
cd /workspace
source devel/setup.bash
rostopic echo /trick/detect/duckie
```
