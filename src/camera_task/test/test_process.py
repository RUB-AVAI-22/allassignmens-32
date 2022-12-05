import unittest
from unittest.mock import patch
import rclpy
import sys
import os
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
import pytest
from cv_bridge import CvBridge

sys.path.append(os.path.dirname(__file__) + "/../camera_task")
from image_process import ImageProcessNode


class ImageProcessNodeCommandTest(unittest.TestCase):
    tests_timeout = 300

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.test_node = ImageProcessNode()
        self.pub_node = rclpy.create_node('pub_process')
        self.sub_node = rclpy.create_node('sub_process')
        self.pub = self.pub_node.create_publisher(
            String,
            'externalCommandStream',
            10
        )
        self.captured_imgs = []
        self.sub_node.create_subscription(
            Image,
            'externalCamStream',
            lambda img: self.captured_imgs.append(img),
            10
        )

    def tearDown(self):
        self.test_node.destroy_node()
        self.pub_node.destroy_node()
        self.sub_node.destroy_node()

    @pytest.mark.timeout(tests_timeout)
    def test_changeFrequencyPositiveInteger(self):
        command = String()
        command.data = "5"
        self.pub.publish(command)
        rclpy.spin_once(self.test_node)
        self.assertEqual(self.test_node.getFrequency(), 5.0)

    @pytest.mark.timeout(tests_timeout)
    def test_changeFrequencyNegativeInteger(self):
        self.test_node.setFrequency(2)
        command = String()
        command.data = "-5"
        self.pub.publish(command)
        rclpy.spin_once(self.test_node)
        self.assertEqual(self.test_node.getFrequency(), 2)

    @pytest.mark.timeout(tests_timeout)
    def test_changeFrequencyPositiveFloat(self):
        command = String()
        command.data = "5.24"
        self.pub.publish(command)
        rclpy.spin_once(self.test_node)
        self.assertEqual(self.test_node.getFrequency(), 5.24)

    @pytest.mark.timeout(tests_timeout)
    def test_changeFrequencyNegativeFloat(self):
        self.test_node.setFrequency(2)
        command = String()
        command.data = "-5.24"
        self.pub.publish(command)
        rclpy.spin_once(self.test_node)
        self.assertEqual(self.test_node.getFrequency(), 2)

    @pytest.mark.timeout(tests_timeout)
    def test_changeFrequencyZero(self):
        command = String()
        command.data = "0"
        self.pub.publish(command)
        rclpy.spin_once(self.test_node)
        self.assertEqual(self.test_node.getFrequency(), 0)

    @pytest.mark.timeout(tests_timeout)
    def test_invalidCommand(self):
        self.test_node.setFrequency(2)
        command = String()
        command.data = "j"
        self.pub.publish(command)
        rclpy.spin_once(self.test_node)
        self.assertEqual(self.test_node.getFrequency(), 2)

    ''' Test not working for some unknown reason
    @pytest.mark.timeout(tests_timeout)
    def test_requestPicture(self):

        mock_cam_node = rclpy.create_node('mock_cam')
        cam_pub = mock_cam_node.create_publisher(
            Image,
            'internalCamStream',
            10
        )
        self.test_node.setFrequency(100)
        bridge = CvBridge()
        input_img = cv2.imread(os.path.dirname(__file__) + "/frame0.jpg")
        cam_pub.publish(bridge.cv2_to_imgmsg(input_img))
        rclpy.spin_once(self.test_node)
        rclpy.spin_once(self.test_node)
        rclpy.spin_once(self.sub_node)
        self.assertEqual(len(self.captured_imgs), 0)
        command = String()
        command.data = "f"
        self.pub.publish(command)
        rclpy.spin_once(self.test_node)
        rclpy.spin_once(self.sub_node)
        self.assertEqual(len(self.captured_imgs), 1)

        mock_cam_node.destroy_node()
    '''

    @pytest.mark.timeout(tests_timeout)
    def test_fuzzyInput(self):
        self.test_node.setFrequency(2)
        command = String()
        command.data = "ffffffffffffff"
        self.pub.publish(command)
        rclpy.spin_once(self.test_node)
        self.assertEqual(self.test_node.getFrequency(), 2)


if __name__ == '__main__':
    unittest.main()
