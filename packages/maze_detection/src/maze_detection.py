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
        
        rospy.logdebug("Received image in cb")
        img = image_msg.data
        header = image_msg.header
        # Decode compressed file
        try:
            bgr = self.bridge.compressed_imgmsg_to_cv2(image_msg)
        except ValueError as e:
            rospy.logerr(f"Could not decode: {e}")
            return
        
        ## Resize image and convert to RGB
        rgb = bgr[..., ::-1]
        rgb = cv2.resize(rgb, (IMAGE_SIZE, IMAGE_SIZE))
        im_pil = Image.fromarray(rgb)

        # Apply edge filter
        im_pil = im_pil.filter(ImageFilter.FIND_EDGES)
        im_pil = im_pil.filter(ImageFilter.SMOOTH_MORE)

        rgb = np.array(im_pil)
        ## Apply to CNN
        bboxes, classes, scores = self.model.predict(im_pil)
        rospy.loginfo(f"bboxes: {bboxes}")
        rospy.loginfo(f"classes: {classes}")
        rospy.loginfo(f"scores: {scores}")

        ## Draw bounding boxes on image and publish
        colors = {0: (0, 255, 255), 1: (0, 165, 255), 2: (0, 250, 0)}
        names = {0: "duckie", 1: "wall", 2: "wall_back"}
        font = cv2.FONT_HERSHEY_SIMPLEX
        for clas, box in zip(classes, bboxes):
            pt1 = np.array([int(box[0]), int(box[1])])
            pt2 = np.array([int(box[2]), int(box[3])])
            pt1 = tuple(pt1)
            pt2 = tuple(pt2)
            color = tuple(reversed(colors[clas]))
            name = names[clas]
            # draw bounding box
            rgb = cv2.rectangle(rgb, pt1, pt2, color, 2)
            # label location
            text_location = (pt1[0], min(pt2[1] + 30, IMAGE_SIZE))
            # draw label underneath the bounding box
            rgb = cv2.putText(rgb, name, text_location, font, 1, color, thickness=2)

        bgr = rgb[..., ::-1]
        obj_det_img = self.bridge.cv2_to_compressed_imgmsg(bgr)
        self.pub_image.publish(obj_det_img)
        
        # Remove low scores
        bboxes = bboxes[scores > 0.5]

    def box_to_pose(self, box):
        """
        Convert bounding box to pose
        """
        pass

    def read_params_from_calibration_file(self):
        """
        Reads the saved parameters from `/data/config/calibrations/camera_intrinsics/DUCKIEBOTNAME.yaml`
        or uses the default values if the file doesn't exist. Adjsuts the ROS paramaters for the
        node with the new values.
        """
        # Check file existence
        cali_file_folder = "/data/config/calibrations/camera_intrinsic/"
        fname = cali_file_folder + self.veh + ".yaml"
        # Use the default values from the config folder if a robot-specific file does not exist.
        if not os.path.isfile(fname):
            fname = cali_file_folder + "default.yaml"
            self.readFile(fname)
            self.logwarn("Camera intrinsics calibration %s not found! Using default instead." % fname)
        else:
            self.readFile(fname)

    def readFile(self, fname):
        with open(fname, "r") as in_file:
            try:
                yaml_dict = yaml.load(in_file, Loader=yaml.FullLoader)
                # self.log(yaml_dict)
                self.image_width = yaml_dict["image_width"]
                self.image_height = yaml_dict["image_height"]
                self.camera_matrix = np.array(yaml_dict["camera_matrix"]["data"]).reshape(3,3)
                self.log(self.camera_matrix)
            except yaml.YAMLError as exc:
                self.logfatal("YAML syntax error. File: %s fname. Exc: %s" % (fname, exc))
                rospy.signal_shutdown("")
                return
            
    def run(self):
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            message = "Hello python"
            rospy.loginfo("pub message: '%s'" % message)
            self.pub.publish(message)
            rate.sleep()

if __name__ == "__main__":
    print("Hello Python")
    name = rospy.get_namespace()
    print(name)
    node = MazeDetectionNode(node_name='camera_control')
    # node.run()
    rospy.spin()