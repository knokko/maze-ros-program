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
SKIP_FRAMES = 5

# Projection matrix created by the minimization of the geometric error
P = np.array([[ 2.12875487e+03,  2.08244026e+03,  2.92247361e+02],
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

        self.pub_pose= rospy.Publisher(
            f"/db4/maze_detection/pose",
            String,
            queue_size=20,
            dt_topic_type=TopicType.PERCEPTION
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
        # Resize image and convert to RGB
        rgb = bgr[..., ::-1]
        # rgb_boxes = rgb
        im_pil = Image.fromarray(rgb)
        im_pil = im_pil.resize((640, 480))
        rgb_boxes = np.array(im_pil)
        # Apply edge filter
        im_pil = im_pil.filter(ImageFilter.FIND_EDGES)
        im_pil = im_pil.filter(ImageFilter.SMOOTH)

        rgb = np.array(im_pil)
        ## Apply to CNN
        bboxes, classes, scores = self.model.predict(im_pil)
        if not bboxes:
            return
        rospy.loginfo(f"bboxes: {bboxes}")

        ## Draw bounding boxes on image and publish
        colors = {0: (0, 255, 255), 1: (0, 165, 255), 2: (0, 250, 0)}
        names = {0: "duckie", 1: "wall", 2: "wall_back"}
        font = cv2.FONT_HERSHEY_SIMPLEX
        detections = []
        # rgb_boxes = self.draw_midline(rgb_boxes)
        for clas, box, score in zip(classes, bboxes, scores):
            if score < 0.3:
                continue
            # Get bounding box coordinates
            pt1 = np.array([int(box[0]), int(box[1])])
            pt2 = np.array([int(box[2]), int(box[3])])
            pt1 = tuple(pt1)
            pt2 = tuple(pt2)

            color = tuple(reversed(colors[clas]))
            # Duckie detection
            name = names[clas]
            if name == "duckie":
                rospy.loginfo("Found duckie!")
            # draw bounding box
            rgb_boxes = cv2.rectangle(rgb_boxes, pt1, pt2, color, 2)
            # Get location, (0,0) is top right in image
            x = (box[0] + box[2]) / 2
            y = box[3]
            
            # Draw mid point via pixel
            rgb_boxes = cv2.circle(rgb_boxes, (int(x), int(y)), 2, (0, 255, 255), thickness=2)
            # label location
            text_location = (pt1[0], min(pt2[1] + 15, IMAGE_SIZE))
            # draw label underneath the bounding box
            rgb_boxes = cv2.putText(rgb_boxes, name, text_location, font, 0.4, color, thickness=2)
            # draw distance in the bounding box
            text_location = (max(pt1[0] - 15, 0), min(pt2[1], IMAGE_SIZE))

            point = self.box_to_pose(box)
            point = self.cartesian2polar(point)
            detections += [(point, name)]
            degree = point[1] * 180 / np.pi
            text = f"S:{score:.2f} r:{point[0]:.2f} rho:{degree:.2f}"
            rgb_boxes = cv2.putText(rgb_boxes, text, text_location, font, 0.3, color, thickness=2)

        # Publish image
        bgr = rgb_boxes[..., ::-1]
        obj_det_img = self.bridge.cv2_to_compressed_imgmsg(bgr)
        self.pub_image.publish(obj_det_img)
        self.publish_object_pose(detections)

    def draw_midline(self, img):
        # Draw midline
        for x in np.arange(0, 2, 0.1):
            for y in np.arange(-2, 2, 0.1):
                l = self.map_world2camera(x, y)
                l = tuple(map(int, l))
                img = cv2.circle(img, l, 1, (100 + int(y)*25, 100 + int(x)*25, 255), thickness=2)
        # Draw null point
        zero = self.map_world2camera(0, 0)
        zero = tuple(map(int, zero))
        print(f"Null point {zero=}")
        img = cv2.circle(img, zero, 10, (255, 255, 0), thickness=2)
        return img

    def box_to_pose(self, box):
        """
        Convert bounding box to pose
        """
        # Find bottom center of bounding box
        x = (box[0] + box[2]) / 2
        y = box[3]

        # Get world point
        world_point = self.pixel2world(x, y)
        self.log(f"from center {(x, y)} to world {world_point}")
        return world_point
    
    def map_world2camera(self, x, y):
        """Map a point in the world to a pixel coordinate"""
        # print(f"In: {x=}, {y=}")
        pixel = np.array([x, y, 1])
        point =  np.dot(P, pixel)
        x, y = point[0] / point[2], point[1] / point[2]
        # print(f"Out: {x=}, {y=}")
        return x, y

    def pixel2world(self, x, y):
        """
        Convert pixel to world coordinates
        """
        pixel = np.array([x, y, 1])
        plane_point = np.linalg.inv(P) @ pixel
        x, y = plane_point[0] / plane_point[2], plane_point[1] / plane_point[2]
        return (x, y)
    
    def publish_object_pose(self, detections):
        """ Publishes the pose of the detected object """
        msg = String()
        for d in detections:
            point, name = d
            msg.data += f"{name},{point[0]},{point[1]},"
        msg.data = msg.data[:-1]
        self.pub_pose.publish(msg)
        self.log(f"Published {msg.data}")

    def cartesian2polar(self, coords):
        """ Transform cartesian to polar coordinates """
        x, y = coords
        rho = np.sqrt(x**2 + y**2)
        phi = np.arctan2(y, x)
        self.log(f"from cartesian {coords} to polar {rho, phi} or {rho, phi * 180 / np.pi}")
        return (rho, phi)

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
    # print("Hello Python")
    # name = rospy.get_namespace()
    # print(name)
    node = MazeDetectionNode(node_name='camera_control')
    rospy.spin()