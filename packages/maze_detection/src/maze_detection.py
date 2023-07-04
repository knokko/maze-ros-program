#!/usr/bin/env python3

import os
import rospy
import yaml
from ros import rostopic, rosgraph
from duckietown.dtros import DTROS, NodeType, TopicType
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage

import cv2
import numpy as np
from cv_bridge import CvBridge
from PIL import Image, ImageFilter

from nn_model.constants import IMAGE_SIZE
from nn_model.model import Wrapper
SKIP_FRAMES = 10

# Projection matrix created by the minimization of the geometric error
R = np.array([[ 2.12875487e+03,  2.08244026e+03,  2.92247361e+02],
       [ 8.23606975e+02,  3.36040459e+01,  4.26124183e+02],
       [ 6.38388920e+00, -9.11630022e-02,  9.18978179e-01]])

class MazeDetectionNode(DTROS):

    init = False
    def __init__(self, node_name):
        super(MazeDetectionNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)
        self.pub = rospy.Publisher('camera_control', String, queue_size=10)
        self.veh = rospy.get_namespace().strip("/")

        self.sub_image = rospy.Subscriber(
            f"/db4/camera_node/image/compressed",
            CompressedImage,
            self.image_cb,
            buff_size=10000000,
            queue_size=1,
        )

        self.pub_image = rospy.Publisher(
            f"/db4/maze_detection/image/compressed",
            CompressedImage,
            queue_size=1,
            dt_topic_type=TopicType.DEBUG
        )

        # Get calibration parameters
        self.read_params_from_calibration_file()

        # Start model
        self.model = Wrapper()
        self.frame_id = 0
        self.bridge = CvBridge()
        rospy.loginfo("Initialized")
        self.init = True


    def image_cb(self, image_msg):
        """
        Callback for image subscriber.
        Takes the image and applies an edge filter on it.
        After this it is applied to a CNN.
        """
        if not self.init:
            return
        
        self.frame_id += 1
        self.frame_id = self.frame_id % (1 + SKIP_FRAMES)
        if self.frame_id != 0:
            return
        
        # Decode compressed file
        try:
            bgr = self.bridge.compressed_imgmsg_to_cv2(image_msg)
        except ValueError as e:
            rospy.logerr(f"Could not decode: {e}")
            return
        original = bgr
        ## Resize image and convert to RGB
        rgb = bgr[..., ::-1]
        rgb = cv2.resize(rgb, (IMAGE_SIZE, IMAGE_SIZE))
        rgb_clone = rgb
        im_pil = Image.fromarray(rgb)

        # Apply edge filter
        im_pil = im_pil.filter(ImageFilter.FIND_EDGES)
        im_pil = im_pil.filter(ImageFilter.SMOOTH_MORE)

        rgb = np.array(im_pil)
        ## Apply to CNN
        bboxes, classes, scores = self.model.predict(im_pil)
        rospy.loginfo(f"bboxes: {bboxes}")

        ## Draw bounding boxes on image and publish
        colors = {0: (0, 255, 255), 1: (0, 165, 255), 2: (0, 250, 0)}
        names = {0: "duckie", 1: "wall", 2: "wall_back"}
        font = cv2.FONT_HERSHEY_SIMPLEX
        for clas, box, score in zip(classes, bboxes, scores):
            if score < 0.3:
                continue
            pt1 = np.array([int(box[0]), int(box[1])])
            pt2 = np.array([int(box[2]), int(box[3])])
            pt1 = tuple(pt1)
            pt2 = tuple(pt2)
            color = tuple(reversed(colors[clas]))
            name = names[clas]
            # draw bounding box
            rgb_clone = cv2.rectangle(rgb_clone, pt1, pt2, color, 2)
            # label location
            text_location = (pt1[0], min(pt2[1] + 30, IMAGE_SIZE))
            # draw label underneath the bounding box
            rgb_clone = cv2.putText(rgb_clone, name, text_location, font, 0.5, color, thickness=2)
            # draw distance in the bounding box
            text_location = (pt1[0], pt1[1])
            point = self.box_to_pose(box)
            self.log(f"{box = } {point = }") 
            text = f"S:{score:.2f} x:{point[0]:.2f} y:{point[1]:.2f}"
            rgb_clone = cv2.putText(rgb_clone, text, text_location, font, 0.5, color, thickness=2)

        rgb_clone = cv2.resize(rgb_clone, (640, 640))
        bgr = rgb_clone[..., ::-1]
        obj_det_img = self.bridge.cv2_to_compressed_imgmsg(bgr)
        # self.pub_image.publish(obj_det_img)
        self.pub_image.publish(self.bridge.cv2_to_compressed_imgmsg(original))

    def box_to_pose(self, box):
        """
        Convert bounding box to pose
        """
        # Get params from box
        x_left, y_bot, x_right, y_top = box
        width = x_right - x_left
        height = y_top - y_bot
        area = width * height
        self.log(f"{width} * {height} = {area}, with {x_left}, {x_right}, {y_top}, {y_bot}")

        center = np.array([x_left + width/2, y_bot + height/2]) / IMAGE_SIZE
        # Get world point
        world_point = self.pixel_to_world(center)
        self.log(f"from center {center} to world {world_point}")
        return world_point

    def pixel_to_world(self, pixel):
        """
        Convert pixel to world coordinates
        """
        camera_matrix = self.homography
        # Get pixel coordinates
        pixel = np.array([pixel[0], pixel[1], 1])
        # Get camera coordinates
        camera_coords = np.linalg.inv(camera_matrix) @ pixel
        # Get world coordinates
        world_coords = np.array([camera_coords[0], camera_coords[1], 1])
        return world_coords

    def read_params_from_calibration_file(self):
        """
        Reads the saved parameters from `/data/config/calibrations/camera_intrinsics/DUCKIEBOTNAME.yaml`
        or uses the default values if the file doesn't exist. Adjsuts the ROS paramaters for the
        node with the new values.
        """
        # Check file existence
        cali_file_folder = "/data/config/calibrations/camera_extrinsic/"
        fname = cali_file_folder + self.veh + ".yaml"
        # Use the default values from the config folder if a robot-specific file does not exist.
        if not os.path.isfile(fname):
            self.logwarn("Camera extrinsic calibration %s not found!" % fname)
            fname = cali_file_folder + "default.yaml"
            self.readFile(fname)
            self.logwarn("Camera extrinsic calibration %s not found! Using default instead." % fname)
        else:
            self.readFile(fname)
            
    def readFile(self, fname):
        with open(fname, "r") as in_file:
            try:
                yaml_dict = yaml.load(in_file, Loader=yaml.FullLoader)
                self.homography = np.array(yaml_dict["homography"]).reshape(3,3)

            except yaml.YAMLError as exc:
                self.logfatal("YAML syntax error. File: %s fname. Exc: %s" % (fname, exc))
                rospy.signal_shutdown("")
                return


if __name__ == "__main__":
    print("Hello Python")
    name = rospy.get_namespace()
    print(name)
    node = MazeDetectionNode(node_name='camera_control')
    rospy.spin()