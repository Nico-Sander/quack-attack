#!/usr/bin/env python3

import os
import cv2
import numpy as np
import rospy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float64

# Deep Learning Imports
import torch
import segmentation_models_pytorch as smp
import torchvision.transforms.functional as TF

class DetectLaneNode:
    def __init__(self, node_name):
        # Initialize the ROS node
        rospy.init_node(node_name)

        # Look 75% down the newly resized 256x256 frame for the center calculation
        self.LANE_SEARCH_Y_RATIO = 0.75
        self._target_im_size = 192 

        self._vehicle_name = os.environ.get("VEHICLE_NAME", "default_robot")

        # Set up deep learning model and device
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        rospy.loginfo(f"[{node_name}] Initializing Segmentation Model on {self.device}...")
        
        self.model = smp.Unet(
            encoder_name="mobilenet_v2", 
            encoder_weights=None, # Weights are loaded from file
            in_channels=3, 
            classes=4,  
        )
        
        # Dynamically resolve the model path based on the node's location
        current_dir = os.path.dirname(os.path.abspath(__file__))
        model_path = os.path.join(current_dir, "../models/lane_segmentation.pth")
        
        self.model.load_state_dict(torch.load(model_path, map_location=self.device))
        self.model.to(self.device)
        self.model.eval()

        # --- THE CPU OPTIMIZATIONS ---
        # 1. Optimize memory layout for CPU
        self.model = self.model.to(memory_format=torch.channels_last)
        
        # 2. Limit threads to prevent CPU thrashing
        torch.set_num_threads(4) 
        
        # 3. JIT Trace the model (requires a dummy pass to compile)
        rospy.loginfo("Compiling model via TorchScript...")
        with torch.no_grad():
            dummy_input = torch.randn(1, 3, self._target_im_size, self._target_im_size).to(self.device)
            dummy_input = dummy_input.to(memory_format=torch.channels_last)
            self.model = torch.jit.trace(self.model, dummy_input)
        rospy.loginfo("Model compiled successfully!")

        # Setup ROS Topics
        self._camera_topic = f"/{self._vehicle_name}/camera_node/image/compressed"
        self.sub_image_original = rospy.Subscriber(
            self._camera_topic, CompressedImage, self.cbFindLane, queue_size=1
        )
        self.pub_lane = rospy.Publisher(
            f"/{self._vehicle_name}/detect/lane", Float64, queue_size=1
        )

        # Init debug channels (Maintaining compatibility with original node)
        self.pub_debug_lane = rospy.Publisher(
            f"/{self._vehicle_name}/debug/lane_croped", CompressedImage, queue_size=1
        )
        self.pub_debug_white = rospy.Publisher(
            f"/{self._vehicle_name}/debug/lane_white", CompressedImage, queue_size=1
        )
        self.pub_debug_yellow = rospy.Publisher(
            f"/{self._vehicle_name}/debug/lane_yellow", CompressedImage, queue_size=1
        )

        self.is_running = False
        self.counter = 0

        rospy.loginfo(f"[{node_name}] Ready and listening to {self._camera_topic}")

    def get_x_from_mask(self, mask, class_idx, fallback_value):
        """
        Extracts the median X coordinate of a specific class (White or Yellow) 
        along a horizontal band defined by LANE_SEARCH_Y_RATIO.
        """
        y_center = int(self._target_im_size * self.LANE_SEARCH_Y_RATIO)
        y_start = y_center - 20
        y_end = y_center + 20

        # Find all X coordinates in the defined Y-band that match the target class
        target_pixels = np.where(mask[y_start:y_end, :] == class_idx)[1]
        
        # If the model sees enough of the line, take the median X position
        if len(target_pixels) > 10:
            return np.median(target_pixels)
        else:
            return fallback_value

    def cbFindLane(self, image_msg):
        # Throttle processing if needed
        if self.counter <= 3:
            self.counter += 1
            return

        if self.is_running:
            return

        self.is_running = True
        self.counter = 0

        # Decode incoming ROS image
        np_arr = np.frombuffer(image_msg.data, np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        # --- 1. PREPROCESSING ---
        # The model was trained on the bottom portion of the camera feed.
        # Crop the bottom 1/3 of the raw image to match the training data format.
        h, w, _ = cv_image.shape
        crop_h = h // 3
        cropped_img = cv_image[h - crop_h:h, 0:w]

        # Convert BGR (OpenCV) to RGB
        rgb_img = cv2.cvtColor(cropped_img, cv2.COLOR_BGR2RGB)

        # 1. Resize using standard OpenCV
        resized_img = cv2.resize(rgb_img, (self._target_im_size, self._target_im_size))

        # 2. Convert to PyTorch Tensor (Automatically converts HWC -> CHW and scales 0-255 -> 0.0-1.0)
        tensor_img = TF.to_tensor(resized_img)

        # 3. Normalize (Using the exact same ImageNet means/stds as before)
        tensor_img = TF.normalize(tensor_img, mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])

        # 4. Add batch dimension and move to device
        tensor_img = tensor_img.unsqueeze(0).to(self.device)

        # ---> STRATEGY 1 OPTIMIZATION: Match the memory format <---
        tensor_img = tensor_img.to(memory_format=torch.channels_last)

        # --- 2. INFERENCE ---
        with torch.no_grad():
            output = self.model(tensor_img)
            # Output shape: [1, 4, H, W]. Argmax gives [H, W] with classes 0, 1, 2, 3
            pred_mask = torch.argmax(output.squeeze(), dim=0).cpu().numpy()

        # --- 3. LANE CENTER CALCULATION ---
        # Class 1 = White, Class 2 = Yellow
        white_alternative = int(self._target_im_size * 0.95)
        yellow_alternative = int(self._target_im_size * 0.05)

        center_white = self.get_x_from_mask(pred_mask, class_idx=1, fallback_value=white_alternative)
        center_yellow = self.get_x_from_mask(pred_mask, class_idx=2, fallback_value=yellow_alternative)

        # Sanity check to prevent lines crossing over each other
        if center_white <= center_yellow:
            if center_white > int(self._target_im_size * 0.4):
                center_yellow = yellow_alternative
            else:
                center_white = white_alternative

        lane_center = (center_white + center_yellow) / 2

        # Error mapping [-1, 1]
        msg_error = Float64()
        msg_error.data = 1 - (lane_center / self._target_im_size * 2)

        self.pub_lane.publish(msg_error)
        print(f"Lane error: {msg_error.data:.4f} range [-1,1]")

        # --- 4. DEBUGGING & DRAWING ---
        # Resize raw cropped image to target size for accurate debug drawing
        resized_bgr = cv2.resize(cropped_img, (self._target_im_size, self._target_im_size))
        self.img = resized_bgr
        
        self.lane_center = lane_center
        self.white_alternative = white_alternative
        self.yellow_alternative = yellow_alternative
        self.center_white = center_white
        self.center_yellow = center_yellow

        # Create binary debug masks for ROS visualization
        self.debug_img_white = (pred_mask == 1).astype(np.uint8) * 255
        self.debug_img_yellow = (pred_mask == 2).astype(np.uint8) * 255

        # Optional: Local cv2.imshow (Can be commented out if running headless)
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

        self.is_running = False

    def run_debug(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():

            # Publish Debug Image with Overlays
            if self.pub_debug_lane.get_num_connections() > 0 and hasattr(self, 'img'):
                debug_img = self.img.copy()
                y_search = int(self._target_im_size * self.LANE_SEARCH_Y_RATIO)
                
                debug_img = cv2.circle(debug_img, (int(self.lane_center), int(self._target_im_size / 2)), 3, (255, 0, 0), -1)
                debug_img = cv2.line(debug_img, (self.white_alternative, 0), (self.white_alternative, self._target_im_size), (255, 255, 255))
                debug_img = cv2.line(debug_img, (self.yellow_alternative, 0), (self.yellow_alternative, self._target_im_size), (255, 255, 0))
                
                debug_img = cv2.line(debug_img, (0, y_search + 20), (self._target_im_size, y_search + 20), (255, 255, 255))
                debug_img = cv2.line(debug_img, (0, y_search - 20), (self._target_im_size, y_search - 20), (255, 255, 255))
                debug_img = cv2.line(debug_img, (int(self._target_im_size / 2), 0), (int(self._target_im_size / 2), self._target_im_size), (0, 255, 0))
                
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
    node = DetectLaneNode("detect_lane_node")
    node.run_debug()
    rospy.spin()