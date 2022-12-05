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
from image_process import ImageProcessNode
from image_display import DisplayNode


class AllNodesTest(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        with patch("cv2.VideoCapture", return_value=cv2.VideoCapture(os.path.dirname(__file__) + "/frame0.jpg")):
            self.cam_node = CameraNode()
        self.input_img = cv2.imread(os.path.dirname(__file__) + "/frame0.jpg")
        self.process_node = ImageProcessNode()
        with patch("cv2.namedWindow", return_value=None):
            with patch("pathlib.Path.mkdir", return_value=None):
                self.display_node = DisplayNode()

    def tearDown(self):
        self.cam_node.destroy_node()
        self.process_node.destroy_node()
        self.display_node.destroy_node()

    def test_walkthroughImage(self):
        rclpy.spin_once(self.cam_node)
        rclpy.spin_once(self.process_node)
        rclpy.spin_once(self.process_node)
        with patch("cv2.imshow", return_value=None):
            with patch("cv2.waitKey", return_value=None):
                with patch("cv2.imwrite", return_value=None):
                    rclpy.spin_once(self.display_node)
        self.assertNotEqual(self.display_node.getLastReceivedFrame().all(), None)


if __name__ == '__main__':
    unittest.main()
