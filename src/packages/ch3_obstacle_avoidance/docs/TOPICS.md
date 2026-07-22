# ROS-Topics

Alle Topics sind mit dem aktuellen `VEHICLE_NAME` namespaced.

Beispiel bei `VEHICLE_NAME=track`:

```text
/track/detect/lane
```

---

## Kamera-Eingang

```text
/<VEHICLE_NAME>/camera_node/image/compressed
```

Wird von `detect_lane_node.py` und `detect_obstacle_node.py` verwendet.

---

## Lane Detection

```text
/<VEHICLE_NAME>/detect/lane
```

Typ: `std_msgs/Float64`  
Normaler Lane-Error für den Controller.

```text
/<VEHICLE_NAME>/detect/lane_borders
```

Typ: `std_msgs/String` mit JSON  
Enthält gelbe und weiße Linienpositionen sowie `yellow_valid` und `white_valid`.

---

## Obstacle Detection

```text
/<VEHICLE_NAME>/detect/duckie
```

Typ: `std_msgs/Float64`  
Legacy-/Kompatibilitätssignal: `1.0`, wenn mindestens ein Duckie im relevanten Bereich erkannt wurde.

```text
/<VEHICLE_NAME>/detect/duckie_BB
```

Typ: `std_msgs/String` mit JSON  
Enthält alle erkannten Duckie-Bounding-Boxes.

---

## Control Output

```text
/<VEHICLE_NAME>/car_cmd_switch_node/cmd
```

Typ: `duckietown_msgs/Twist2DStamped`  
Fahrbefehl des Controllers.

---

## Debug

```text
/<VEHICLE_NAME>/debug/free_path_plan
```

JSON-Debugdaten aus dem Controller. Wird vom Dashboard für das Planning-Overlay genutzt.

```text
/<VEHICLE_NAME>/debug/obstacle_detection
```

YOLO-Debugbild mit Bounding-Boxes.

```text
/<VEHICLE_NAME>/debug/lane_croped
/<VEHICLE_NAME>/debug/lane_white
/<VEHICLE_NAME>/debug/lane_yellow
```

Lane-Debugbilder.
