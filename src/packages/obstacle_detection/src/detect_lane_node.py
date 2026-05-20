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
        self.lane_search_y_ratio = 0.25
        self._target_im_size = 192 
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

        self.duckies = []
        self.last_duckie_time = 0.0
        self.duckie_mask_hold_time = 0.6
        self.duckie_mask_margin = 0.04
        self.duckie_min_area_px = 80

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
        self.sub_obstacles = rospy.Subscriber(
            f"{base_topic}/detect/duckie_BB",
            String,
            self.cbObstacles,
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

        rospy.loginfo(f"[{node_name}] Ready and listening to {base_topic}/camera_nod/image/compressed")

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
        model_path = os.path.join(current_dir, "../models/lane_segmentation.pth")

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

    @staticmethod
    def clamp01(value):
        return max(0.0, min(1.0, float(value)))

    def cbObstacles(self, msg):
        """Stores the latest duckie boxes for segmentation-mask cleanup."""
        try:
            data = json.loads(msg.data)
            raw_duckies = data.get("duckies", [])
            if not raw_duckies and data.get("detected", False):
                if data.get("class_name", "").lower() == "duckie":
                    raw_duckies = [data]
            self.duckies = [d for d in raw_duckies if d.get("class_name", "").lower() == "duckie"]
            if self.duckies:
                self.last_duckie_time = rospy.Time.now().to_sec()
        except Exception as e:
            rospy.logwarn_throttle(1.0, f"Could not parse duckie_BB in lane node: {e}")

    def duckies_are_recent(self):
        if not self.duckies:
            return False
        return (rospy.Time.now().to_sec() - self.last_duckie_time) < self.duckie_mask_hold_time

    def remove_duckies_from_mask(self, pred_mask):
        """Removes YOLO duckie boxes from the segmentation mask.

        The lane model works on the lower third of the camera image resized to
        192x192. YOLO boxes are normalized to the full camera image, so the y
        coordinates are mapped into this crop before pixels are cleared.
        """
        if not self.duckies_are_recent():
            return pred_mask

        cleaned = pred_mask.copy()
        h_mask, w_mask = cleaned.shape[:2]
        crop_start_y = 2.0 / 3.0
        crop_end_y = 1.0
        crop_height = crop_end_y - crop_start_y

        for duckie in self.duckies:
            try:
                xmin = float(duckie.get("xmin", duckie["x_center"] - duckie["width"] / 2.0))
                xmax = float(duckie.get("xmax", duckie["x_center"] + duckie["width"] / 2.0))
                ymin = float(duckie.get("ymin", duckie["y_center"] - duckie["height"] / 2.0))
                ymax = float(duckie.get("ymax", duckie["y_center"] + duckie["height"] / 2.0))
            except Exception:
                continue

            xmin = self.clamp01(xmin - self.duckie_mask_margin)
            xmax = self.clamp01(xmax + self.duckie_mask_margin)
            ymin = self.clamp01(ymin - self.duckie_mask_margin)
            ymax = self.clamp01(ymax + self.duckie_mask_margin)

            if ymax < crop_start_y:
                continue

            ymin_crop = (max(ymin, crop_start_y) - crop_start_y) / crop_height
            ymax_crop = (min(ymax, crop_end_y) - crop_start_y) / crop_height
            if ymax_crop <= 0.0 or ymin_crop >= 1.0:
                continue

            x1 = int(self.clamp01(xmin) * w_mask)
            x2 = int(self.clamp01(xmax) * w_mask)
            y1 = int(self.clamp01(ymin_crop) * h_mask)
            y2 = int(self.clamp01(ymax_crop) * h_mask)

            if x2 <= x1 or y2 <= y1:
                continue
            #if (x2 - x1) * (y2 - y1) < self.duckie_min_area_px:
            #    continue

            cleaned[y1:y2, x1:x2] = 0

        return cleaned

    def get_x_from_mask(self, mask, class_idx, fallback_value):
        y_center = int(self._target_im_size * self.lane_search_y_ratio)
        y_start = y_center - 20
        y_end = y_center + 20

        target_pixels = np.where(mask[y_start:y_end, :] == class_idx)[1]

        if len(target_pixels) > 10:
            return np.median(target_pixels), True

        return fallback_value, False

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

        pred_mask = self.remove_duckies_from_mask(pred_mask)

        # 3. Lane Center Calculation (Class 1 = White, Class 2 = Yellow) 
        center_white, white_valid = self.get_x_from_mask(
            pred_mask,
            class_idx=1,
            fallback_value=self.white_fallback
        )

        center_yellow, yellow_valid = self.get_x_from_mask(
            pred_mask,
            class_idx=2,
            fallback_value=self.yellow_fallback
        )

        # Sanity check for crossed lines
        if center_white <= center_yellow:
            if white_valid and not yellow_valid:
                center_yellow = self.yellow_fallback
            elif yellow_valid and not white_valid:
                center_white = self.white_fallback
            elif center_white > int(self._target_im_size * 0.4):
                center_yellow = self.yellow_fallback
                yellow_valid = False
            else:
                center_white = self.white_fallback
                white_valid = False

        lane_center = (center_white + center_yellow) / 2.0

        # Map center to an error [-1, 1] and publish
        msg_error = Float64()
        msg_error.data = 1.0 - (lane_center / self._target_im_size * 2.0)
        self.pub_lane.publish(msg_error)
        #print(f"Lane error: {msg_error.data:.4f} range [-1,1]")

        lane_borders_msg = String()

        lane_borders_msg.data = json.dumps({
            "yellow_x": float(center_yellow / self._target_im_size),
            "white_x": float(center_white / self._target_im_size),
            "lane_center_x": float(lane_center / self._target_im_size),
            "yellow_valid": bool(yellow_valid),
            "white_valid": bool(white_valid),
            "valid": bool(center_white > center_yellow and (yellow_valid or white_valid))
        })

        self.pub_lane_borders.publish(lane_borders_msg)

        # 4. Save state for debugging thread
        resized_bgr = cv2.resize(cropped_img, (self._target_im_size, self._target_im_size))
        self.img = resized_bgr
        
        self.lane_center = lane_center
        self.center_white = center_white
        self.center_yellow = center_yellow

        # Create binary debug masks for ROS visualization
        self.debug_img_white = (pred_mask == 1).astype(np.uint8) * 255
        self.debug_img_yellow = (pred_mask == 2).astype(np.uint8) * 255

        # Optional: Local cv2.imshow (Can be commented out if running headless)
        '''
        try:
            display_img = self.img.copy()
            y_search = int(self._target_im_size * self.LANE_SEARCH_Y_RATIO)
            
            # Draw calculated points
            cv2.circle(display_img, (int(lane_center), int(self._target_im_size / 2)), 3, (255, 0, 0), -1)
            cv2.circle(display_img, (int(center_white), y_search), 5, (255, 255, 255), -1)
            cv2.circle(display_img, (int(center_yellow), y_search), 5, (0, 255, 255), -1)
            
            # Draw search boundaries
            cv2.line(display_img, (0, y_search + 20), (self._target_im_size, y_search + 20), (255, 255, 255))
            cv2.line(display_img, (0, y_search - 20), (self._target_im_size, y_search - 20), (255, 255, 255))
            cv2.line(display_img, (int(self._target_im_size / 2), 0), (int(self._target_im_size / 2), self._target_im_size), (0, 255, 0))

            cv2.imshow("Semantic Lane Detection", display_img)
            cv2.waitKey(1)
        except Exception:
            pass # Ignore if X11 forwarding isn't setup in the Docker container
        '''
        
        end_time = rospy.Time().now().to_sec()
        rospy.loginfo(f"Inf Time: {end_time - start_time}")

        self.is_running = False

    def run_debug(self):
        """Loops continuously to publish debug visualizers if requested."""
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            # Publish Debug Image with Overlays
            if self.pub_debug_lane.get_num_connections() > 0 and hasattr(self, 'img'):
                debug_img = self.img.copy()
                y_search = int(self._target_im_size * self.lane_search_y_ratio)
                
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

            rate.sleep()


if __name__ == "__main__":
    try:
        node = DetectLaneNode("detect_lane_node")
        node.run_debug()
    except rospy.ROSInterruptException:
        pass