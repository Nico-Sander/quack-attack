#!/usr/bin/env python3

import os
import json
import time

import cv2
import numpy as np
import rospy
import torch
from ultralytics import YOLO

from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float64, String


class DetectObstacleNode:
    def __init__(self, node_name):
        rospy.init_node(node_name)

        self.node_name = node_name
        self.vehicle_name = os.environ.get("VEHICLE_NAME", "default_robot")

        self.frame_counter = 0
        self.model = None
        self.classes = []

        self.load_config()
        self.load_model()

        base_topic = f"/{self.vehicle_name}"

        self.sub_image = rospy.Subscriber(
            f"{base_topic}/camera_node/image/compressed",
            CompressedImage,
            self.cb_process_image,
            queue_size=1
        )

        # Einfaches Signal für switch_control_node:
        # 0.0 = kein Hindernis auf Spur
        # 1.0 = Hindernis auf Spur
        self.pub_duckie = rospy.Publisher(
            f"{base_topic}/detect/duckie",
            Float64,
            queue_size=1
        )

        # Strukturierte Detektionsinfo als JSON
        self.pub_obstacle = rospy.Publisher(
            f"{base_topic}/detect/duckie_BB",
            String,
            queue_size=1
        )

        # Debug-Bild mit Hindernisregion und Bounding Boxes
        self.pub_debug = rospy.Publisher(
            f"{base_topic}/debug/obstacle_detection",
            CompressedImage,
            queue_size=1
        )

        rospy.loginfo(f"[{node_name}] started.")
        rospy.loginfo(f"[{node_name}] subscribing to {base_topic}/camera_node/image/compressed")

    def load_config(self):
        current_dir = os.path.dirname(os.path.abspath(__file__))
        config_path = os.path.join(current_dir, "../config/detect_obstacle_node.json")

        if not os.path.exists(config_path):
            raise FileNotFoundError(f"Config file not found: {config_path}")

        with open(config_path, "r") as f:
            config = json.load(f)

        params = config["parameters"]

        self.confidence_threshold = params["model"]["confidence_threshold"]["default"]
        self.process_every_n_frames = int(params["model"]["process_every_n_frames"]["default"])
        self.input_size = int(params["model"]["input_size"]["default"])

        self.x_min = float(params["obstacle_region"]["x_min"]["default"])
        self.x_max = float(params["obstacle_region"]["x_max"]["default"])
        self.y_min = float(params["obstacle_region"]["y_min"]["default"])

        rospy.loginfo(
            f"[{self.node_name}] Config loaded: "
            f"conf={self.confidence_threshold}, "
            f"every_n={self.process_every_n_frames}, "
            f"input_size={self.input_size}, "
            f"region=({self.x_min}, {self.x_max}, {self.y_min})"
        )

    def load_classes(self):
        current_dir = os.path.dirname(os.path.abspath(__file__))
        classes_path = os.path.join(current_dir, "../models/classes.txt")

        if not os.path.exists(classes_path):
            rospy.logwarn(f"[{self.node_name}] classes.txt not found at {classes_path}")
            self.classes = []
            return

        with open(classes_path, "r") as f:
            self.classes = [line.strip() for line in f.readlines() if line.strip()]

        rospy.loginfo(f"[{self.node_name}] Loaded classes: {self.classes}")

    def load_model(self):
        current_dir = os.path.dirname(os.path.abspath(__file__))
        
        model_path = os.path.join(current_dir, "../models/YOLOv11_duckie_detection_modell.pt")

        if not os.path.exists(model_path):
            rospy.logwarn(
                f"[{self.node_name}] No model found at {model_path}. "
                f"Using mock detection."
            )
            self.model = None
            return

        rospy.loginfo(f"[{self.node_name}] Loading YOLOv11 model from {model_path}")

        try:
            # YOLOv11 Modell direkt laden
            self.model = YOLO(model_path)
            
            # Die Klassen direkt aus dem Modell auslesen (macht classes.txt überflüssig)
            self.classes = list(self.model.names.values())

            # GPU nutzen, falls vorhanden (Ultralytics macht das oft automatisch, 
            # aber wir loggen es zur Sicherheit)
            if torch.cuda.is_available():
                rospy.loginfo(f"[{self.node_name}] CUDA available. Using GPU.")
            else:
                rospy.loginfo(f"[{self.node_name}] CUDA not available. Using CPU.")

            rospy.loginfo(f"[{self.node_name}] YOLOv11 model loaded successfully. Classes: {self.classes}")

        except Exception as e:
            rospy.logerr(f"[{self.node_name}] Could not load YOLOv11 model: {e}")
            self.model = None

    def cb_process_image(self, image_msg):
        self.frame_counter += 1
        if self.process_every_n_frames > 1:
            if self.frame_counter % self.process_every_n_frames != 0:
                return

        np_arr = np.frombuffer(image_msg.data, np.uint8)
        img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        if img is None:
            rospy.logwarn_throttle(1.0, f"[{self.node_name}] Could not decode image.")
            return

        detections = self.detect_with_model(img)

        obstacle_on_lane = False
        best_detection = None

        for det in detections:
            if det["confidence"] < self.confidence_threshold:
                continue

            class_name = det["class_name"].lower()

            # Für Challenge 3/5 relevant.
            # Andere Klassen wie signs oder traffic_light werden erkannt,
            # sollen hier aber nicht als Hindernis auf der Fahrspur zählen.
            if class_name not in ["duckie", "duckiebot"]:
                continue

            x_center = det["x_center"]
            y_center = det["y_center"]

            if self.x_min <= x_center <= self.x_max and y_center >= self.y_min:
                obstacle_on_lane = True
                best_detection = det
                break

        # Falls kein Objekt auf der Spur liegt, aber trotzdem etwas erkannt wurde,
        # wird die beste Erkennung für /detect/obstacle ausgegeben.
        if best_detection is None and len(detections) > 0:
            best_detection = max(detections, key=lambda d: d["confidence"])

        duckie_msg = Float64()
        duckie_msg.data = 1.0 if obstacle_on_lane else 0.0
        self.pub_duckie.publish(duckie_msg)

        if best_detection is not None:
            output = best_detection.copy()
            output["detected"] = True
            output["obstacle_on_lane"] = obstacle_on_lane
        else:
            output = {
                "detected": False,
                "class_name": "",
                "confidence": 0.0,
                "x_center": 0.0,
                "y_center": 0.0,
                "width": 0.0,
                "height": 0.0,
                "obstacle_on_lane": False
            }

        self.pub_obstacle.publish(String(data=json.dumps(output)))
        self.publish_debug_image(img, detections, obstacle_on_lane)

    def detect_with_model(self, img):
        if self.model is None:
            return self.mock_detect(img)

        try:
            start_time = time.time()
            results = self.model(img, imgsz=self.input_size, verbose=False)
            inference_time_ms = (time.time() - start_time) * 1000.0

            rospy.loginfo_throttle(
                1.0,
                f"[{self.node_name}] YOLO inference time: {inference_time_ms:.1f} ms"
            )
        except Exception as e:
            rospy.logerr_throttle(1.0, f"[{self.node_name}] YOLOv11 inference failed: {e}")
            return []

        detections = []
        h, w = img.shape[:2]

        try:
            boxes = results[0].boxes
        except Exception as e:
            rospy.logerr_throttle(1.0, f"[{self.node_name}] Could not parse YOLOv11 output: {e}")
            return []

        for box in boxes:
            # Konfidenzwert auslesen (als float)
            conf = float(box.conf[0].cpu().numpy())
            
            # Filtern: Wenn die Erkennung unsicher ist, ignorieren wir sie direkt
            if conf < self.confidence_threshold:
                continue

            # Koordinaten und Klassen-ID auslesen
            xmin, ymin, xmax, ymax = box.xyxy[0].cpu().numpy()
            cls_id = int(box.cls[0].cpu().numpy())
            
            # Klassenname aus dem Modell holen (z.B. "Duckie")
            class_name = self.model.names.get(cls_id, str(cls_id))

            # In relative Werte (0.0 bis 1.0) umrechnen, genau wie dein alter Code es erwartet
            x_center = ((xmin + xmax) / 2.0) / w
            y_center = ((ymin + ymax) / 2.0) / h
            box_width = (xmax - xmin) / w
            box_height = (ymax - ymin) / h

            detections.append({
                "class_name": class_name,
                "confidence": conf,
                "x_center": float(x_center),
                "y_center": float(y_center),
                "width": float(box_width),
                "height": float(box_height)
            })

        return detections

    def mock_detect(self, img):
        """
        Fallback, falls Modell oder YOLOv5-Repo nicht gefunden wird.
        Gibt standardmäßig keine Erkennung zurück.

        Rückgabeformat:
        [
            {
                "class_name": "duckie",
                "confidence": 0.85,
                "x_center": 0.50,
                "y_center": 0.65,
                "width": 0.20,
                "height": 0.25
            }
        ]

        Alle Werte x/y/width/height sind relativ zum Bildbereich [0.0, 1.0].
        """
        return []

    def publish_debug_image(self, img, detections, obstacle_on_lane):
        debug_img = img.copy()
        h, w = debug_img.shape[:2]

        # Bereich zeichnen, in dem ein Objekt als "auf der Spur" gewertet wird.
        x1 = int(self.x_min * w)
        x2 = int(self.x_max * w)
        y1 = int(self.y_min * h)
        y2 = h

        cv2.rectangle(debug_img, (x1, y1), (x2, y2), (255, 255, 0), 2)

        for det in detections:
            cx = det["x_center"]
            cy = det["y_center"]
            bw = det["width"]
            bh = det["height"]

            bx1 = int((cx - bw / 2.0) * w)
            by1 = int((cy - bh / 2.0) * h)
            bx2 = int((cx + bw / 2.0) * w)
            by2 = int((cy + bh / 2.0) * h)

            # Begrenzen, damit OpenCV keine komischen Werte bekommt
            bx1 = max(0, min(w - 1, bx1))
            by1 = max(0, min(h - 1, by1))
            bx2 = max(0, min(w - 1, bx2))
            by2 = max(0, min(h - 1, by2))

            class_name = det["class_name"]
            confidence = det["confidence"]
            label = f"{class_name} {confidence:.2f}"

            class_name_lower = class_name.lower()
            is_obstacle_class = class_name_lower in ["duckie", "duckiebot"]
            is_inside_region = (
                self.x_min <= det["x_center"] <= self.x_max and
                det["y_center"] >= self.y_min
            )

            if is_obstacle_class and is_inside_region:
                color = (0, 0, 255)
            else:
                color = (0, 255, 0)

            cv2.rectangle(debug_img, (bx1, by1), (bx2, by2), color, 2)
            cv2.putText(
                debug_img,
                label,
                (bx1, max(by1 - 5, 20)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                color,
                2
            )

        status = "OBSTACLE ON LANE" if obstacle_on_lane else "clear"
        status_color = (0, 0, 255) if obstacle_on_lane else (0, 255, 0)

        cv2.putText(
            debug_img,
            status,
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            status_color,
            2
        )

        debug_msg = CompressedImage()
        debug_msg.header.stamp = rospy.Time.now()
        debug_msg.format = "jpeg"
        debug_msg.data = np.array(cv2.imencode(".jpg", debug_img)[1]).tobytes()
        self.pub_debug.publish(debug_msg)


if __name__ == "__main__":
    try:
        node = DetectObstacleNode("detect_obstacle_node")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass