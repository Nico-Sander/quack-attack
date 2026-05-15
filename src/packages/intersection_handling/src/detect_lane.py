#!/usr/bin/env python3

"""
ROS 1 node for semantic lane detection using a PyTorch U-Net model.
Outputs a cross-track error to keep the vehicle centered.
"""

import os
import json
import cv2
import numpy as np
import rospy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float64, String
import torch
import segmentation_models_pytorch as smp
import torchvision.transforms.functional as TF

class DetectLaneNode:
    """Deep learning node for lane segmentation and error calculation."""

    def __init__(self, node_name):
        rospy.init_node(node_name)

        # Configuration Constants
        self.config = self._load_config()
        self.lane_search_y_ratio = self.config["lane_search_y_ratio"]
        self._target_im_size = self.config["target_im_size"]
        self.current_dynamic_y = self._target_im_size * self.lane_search_y_ratio
        
        self._vehicle_name = os.environ.get("VEHICLE_NAME", "default_robot")

        # State Variables
        self.is_running = False
        self.counter = 0

        # Debug State Variables
        self.debug_img_bgr = None
        self.debug_mask_white = None
        self.debug_mask_yellow = None
        self.lane_center = 0.0
        self.center_white = 0.0
        self.center_yellow = 0.0
        self.white_fallback = int(self._target_im_size * 0.95)
        self.yellow_fallback = int(self._target_im_size * 0.05)

        # Initialize PyTorch Model
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        rospy.loginfo(f"[{node_name}] Initializing Segmentation Model on {self.device}...")
        self._init_model()
        
        # Setup ROS Topics
        base_topic = f"/{self._vehicle_name}"

        self.sub_image_original = rospy.Subscriber(
            f"{base_topic}/camera_node/image/compressed", 
            CompressedImage, 
            self.cbFindLane, 
            queue_size=1
        )
        self.pub_lane = rospy.Publisher(
            f"{base_topic}/detect/lane", Float64, queue_size=1
        )

        self.pub_lane_borders = rospy.Publisher(
            f"{base_topic}/detect/lane_borders", 
            String, 
            queue_size=1
        )

        # Debug Publishers
        self.pub_debug_lane = rospy.Publisher(
            f"{base_topic}/debug/lane_croped", CompressedImage, queue_size=1
        )
        self.pub_debug_white = rospy.Publisher(
            f"{base_topic}/debug/lane_white", CompressedImage, queue_size=1
        )
        self.pub_debug_yellow = rospy.Publisher(
            f"{base_topic}/debug/lane_yellow", CompressedImage, queue_size=1
        )
        self.pub_debug_red = rospy.Publisher(
            f"{base_topic}/debug/lane_red", CompressedImage, queue_size=1
        )

        rospy.loginfo(f"[{node_name}] Ready and listening to {base_topic}/camera_node/image/compressed")

    def _load_config(self):
        """Loads configuration parameters from JSON."""
        config_path = os.path.join(os.path.dirname(__file__), "../config/config.json")
        try:
            with open(config_path, "r") as f:
                return json.load(f)["detect_lane"]
        except (FileNotFoundError, KeyError) as e:
            rospy.logwarn(f"Using default detect_lane config due to: {e}")
            return {
                "lane_search_y_ratio": 0.25,
                "target_im_size": 192
            }

    def _init_model(self):
        """Loads and optimizes the U-Net model for inference."""
        self.model = smp.Unet(
            encoder_name="mobilenet_v2", 
            encoder_weights=None, # Weights are loaded from file
            in_channels=3, 
            classes=4,  
        )

        # Load weights
        current_dir = os.path.dirname(os.path.abspath(__file__))
        model_path = os.path.join(current_dir, "../models/lane_segmentation_002_model.pth")

        if not os.path.exists(model_path):
            rospy.logerr(f"Model weights not found at {model_path}")
            return

        self.model.load_state_dict(torch.load(model_path, map_location=self.device))
        self.model.to(self.device)
        self.model.eval()

        # CPU Optimizations
        self.model = self.model.to(memory_format=torch.channels_last)
        torch.set_num_threads(4) 
        
        # JIT Traceing compilation
        rospy.loginfo("Compiling model via TorchScript...")
        with torch.no_grad():
            dummy_input = torch.randn(1, 3, self._target_im_size, self._target_im_size).to(self.device)
            dummy_input = dummy_input.to(memory_format=torch.channels_last)
            self.model = torch.jit.trace(self.model, dummy_input)
        rospy.loginfo("Model compiled successfully!")

    def get_x_from_mask(self, mask, class_idx, fallback_value, search_y_center):
        """
        Extracts the median X coordinate of a specific class along a defined horizontal band.
        Now uses a dynamic search_y_center to avoid looking past red lines.
        """
        y_start = int(search_y_center - 15)
        y_end = int(search_y_center + 15)
        
        # Ensure we don't go out of bounds
        y_start = max(0, y_start)
        y_end = min(self._target_im_size, y_end)

        # Find X coordinates matching the target class within the Y-band
        target_pixels = np.where(mask[y_start:y_end, :] == class_idx)[1]
        
        if len(target_pixels) > 10:
            return np.median(target_pixels)

        return fallback_value

    def cbFindLane(self, image_msg):
        """Processes incoming camera frames to calculate lane error."""

        # Start inference timer
        start_time = rospy.Time().now().to_sec()

        # Throttle processing if needed
        if self.counter <= 1:
            self.counter += 1
            return

        if self.is_running:
            return

        self.is_running = True
        self.counter = 0

        # Decode Image
        np_arr = np.frombuffer(image_msg.data, np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        if cv_image is None:
            self.is_running = False
            return

        # 1. Preprocessing
        # The model was trained on the bottom 1/3 of the camera feed.
        h, w, _ = cv_image.shape
        crop_h = h // 3
        cropped_img = cv_image[h - crop_h:h, 0:w]

        # Convert BGR (OpenCV) to RGB
        rgb_img = cv2.cvtColor(cropped_img, cv2.COLOR_BGR2RGB)

        # Resize using standard OpenCV
        resized_img = cv2.resize(rgb_img, (self._target_im_size, self._target_im_size))

        # Convert to PyTorch Tensor (Automatically converts HWC -> CHW and scales 0-255 -> 0.0-1.0)
        tensor_img = TF.to_tensor(resized_img)

        # Normalize (Using the exact same ImageNet means/stds as in training)
        tensor_img = TF.normalize(tensor_img, mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])

        # Add batch dimension and move to device
        tensor_img = tensor_img.unsqueeze(0).to(self.device)

        # Match the memory format
        tensor_img = tensor_img.to(memory_format=torch.channels_last)

        # 2. Inference
        with torch.no_grad():
            output = self.model(tensor_img)
            pred_mask = torch.argmax(output.squeeze(), dim=0).cpu().numpy()

        mask_red = (pred_mask == 3).astype(np.uint8)
        y_coords_red, x_coords_red = np.where(mask_red > 0)

        closest_red_y = -1.0
        red_angle = 0.0
        red_detected = False

        if len(y_coords_red) > 50:
            red_detected = True
            closest_red_y = float(np.max(y_coords_red))

            # Fit a line to the red pixels to find the angle
            m, c = np.polyfit(x_coords_red, y_coords_red, 1)

            # Convert slope to angle in radians
            red_angle = float(np.arctan(m))

        default_y_center = self._target_im_size * self.lane_search_y_ratio
        dynamic_y_center = default_y_center

        if red_detected:
            # If red line is lower in the image that the default search band
            if closest_red_y > default_y_center:
                dynamic_y_center = closest_red_y + ((self._target_im_size - closest_red_y) / 2.0)

        # 3. Lane Center Calculation (Class 1 = White, Class 2 = Yellow) 
        center_white = self.get_x_from_mask(pred_mask, class_idx=1, fallback_value=self.white_fallback, search_y_center=dynamic_y_center)
        center_yellow = self.get_x_from_mask(pred_mask, class_idx=2, fallback_value=self.yellow_fallback, search_y_center=dynamic_y_center)

        # Sanity check for crossed lines
        if center_white <= center_yellow:
            if center_white > int(self._target_im_size * 0.4):
                center_yellow = self.yellow_fallback
            else:
                center_white = self.white_fallback

        lane_center = (center_white + center_yellow) / 2.0

        # Map center to an error [-1, 1] and publish
        msg_error = Float64()
        msg_error.data = 1.0 - (lane_center / self._target_im_size * 2.0)
        self.pub_lane.publish(msg_error)
        
        # Publish rich JSON staet including red line geometry
        lane_borders_msg = String()
        lane_borders_msg.data = json.dumps({
            "yellow_x": float(center_yellow / self._target_im_size),
            "white_x": float(center_white / self._target_im_size),
            "lane_center_x": float(lane_center / self._target_im_size),
            "valid_lanes": bool(center_white > center_yellow),
            "red_detected": red_detected,
            "red_distance_y": float(closest_red_y / self._target_im_size), # Normalized [0, 1]
            "red_angle": red_angle
        })

        self.pub_lane_borders.publish(lane_borders_msg)

        # 4. Save state for debugging thread
        resized_bgr = cv2.resize(cropped_img, (self._target_im_size, self._target_im_size))
        self.img = resized_bgr
        
        self.lane_center = lane_center
        self.center_white = center_white
        self.center_yellow = center_yellow
        self.current_dynamic_y = dynamic_y_center

        # Create binary debug masks for ROS visualization
        self.debug_img_white = (pred_mask == 1).astype(np.uint8) * 255
        self.debug_img_yellow = (pred_mask == 2).astype(np.uint8) * 255
        self.debug_img_red = (pred_mask == 3).astype(np.uint8) * 255

        end_time = rospy.Time().now().to_sec()

        self.is_running = False

    def run_debug(self):
        """Loops continuously to publish debug visualizers if requested."""
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            # Publish Debug Image with Overlays
            if self.pub_debug_lane.get_num_connections() > 0 and hasattr(self, 'img'):
                debug_img = self.img.copy()
                y_search = int(self.current_dynamic_y)
                
                # Draw centers and boundaries
                debug_img = cv2.circle(debug_img, (int(self.lane_center), int(self._target_im_size / 2)), 3, (255, 0, 0), -1)
                debug_img = cv2.line(debug_img, (self.white_fallback, 0), (self.white_fallback, self._target_im_size), (255, 255, 255))
                debug_img = cv2.line(debug_img, (self.yellow_fallback, 0), (self.yellow_fallback, self._target_im_size), (255, 255, 0))
                
                # Draw search window
                debug_img = cv2.line(debug_img, (0, y_search + 20), (self._target_im_size, y_search + 20), (255, 255, 255))
                debug_img = cv2.line(debug_img, (0, y_search - 20), (self._target_im_size, y_search - 20), (255, 255, 255))
                debug_img = cv2.line(debug_img, (int(self._target_im_size / 2), 0), (int(self._target_im_size / 2), self._target_im_size), (0, 255, 0))
                
                # Draw detected points
                debug_img = cv2.circle(debug_img, (int(self.center_white), y_search), 5, (255, 255, 255), -1)
                debug_img = cv2.circle(debug_img, (int(self.center_yellow), y_search), 5, (0, 255, 255), -1)

                debug_msg = CompressedImage()
                debug_msg.header.stamp = rospy.Time.now()
                debug_msg.format = "jpeg"
                debug_msg.data = np.array(cv2.imencode(".jpg", debug_img)[1]).tobytes()
                self.pub_debug_lane.publish(debug_msg)

            # Publish raw isolated White Mask
            if self.pub_debug_white.get_num_connections() > 0 and hasattr(self, 'debug_img_white'):
                debug_msg = CompressedImage()
                debug_msg.header.stamp = rospy.Time.now()
                debug_msg.format = "jpeg"
                debug_msg.data = np.array(cv2.imencode(".jpg", self.debug_img_white)[1]).tobytes()
                self.pub_debug_white.publish(debug_msg)

            # Publish raw isolated Yellow Mask
            if self.pub_debug_yellow.get_num_connections() > 0 and hasattr(self, 'debug_img_yellow'):
                debug_msg = CompressedImage()
                debug_msg.header.stamp = rospy.Time.now()
                debug_msg.format = "jpeg"
                debug_msg.data = np.array(cv2.imencode(".jpg", self.debug_img_yellow)[1]).tobytes()
                self.pub_debug_yellow.publish(debug_msg)

           # Publish raw isolated Red Mask
            if self.pub_debug_red.get_num_connections() > 0 and hasattr(self, 'debug_img_red'):
                debug_msg = CompressedImage()
                debug_msg.header.stamp = rospy.Time.now()
                debug_msg.format = "jpeg"
                debug_msg.data = np.array(cv2.imencode(".jpg", self.debug_img_red)[1]).tobytes()
                self.pub_debug_red.publish(debug_msg)

            rate.sleep()


if __name__ == "__main__":
    try:
        node = DetectLaneNode("detect_lane_node")
        node.run_debug()
    except rospy.ROSInterruptException:
        pass