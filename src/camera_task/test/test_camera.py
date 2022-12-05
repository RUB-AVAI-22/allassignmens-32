import unittest
from unittest.mock import patch
import rclpy
import sys
import os
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

sys.path.append(os.path.dirname(__file__) + "/../camera_task")
from camera import CameraNode


class CameraNodeCaptureImageTest(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        with patch("cv2.VideoCapture", return_value=cv2.VideoCapture(os.path.dirname(__file__) + "/frame0.jpg")):
            self.test_node = CameraNode()
        self.input_img = cv2.imread(os.path.dirname(__file__) + "/frame0.jpg")

    def tearDown(self):
        self.test_node.destroy_node()

    def test_readFrame(self):
        success, captured_img = self.test_node.getVideoFrame()
        self.assertEqual(success, True)
        self.assertEqual(captured_img.all(), self.input_img.all())


class CameraNodePublishImageTest(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        with patch("cv2.VideoCapture", return_value=cv2.VideoCapture(os.path.dirname(__file__) + "/frame0.jpg")):
            self.test_node = CameraNode()
        self.sub_node = rclpy.create_node('sub_camera')
        self.captured_imgmsg = []
        self.sub_node.create_subscription(
            Image,
            'internalCamStream',
            lambda img: self.captured_imgmsg.append(img),
            10
        )
        self.input_img = cv2.imread(os.path.dirname(__file__) + "/frame0.jpg")
        self.bridge = CvBridge()

    def tearDown(self):
        self.test_node.destroy_node()
        self.sub_node.destroy_node()

    def test_publishImage(self):
        rclpy.spin_once(self.test_node)
        rclpy.spin_once(self.sub_node)
        self.assertEqual(len(self.captured_imgmsg), 1)
        captured_img = self.bridge.imgmsg_to_cv2(self.captured_imgmsg[0])
        self.assertEqual(captured_img.all(), self.input_img.all())


if __name__ == '__main__':
    unittest.main()
