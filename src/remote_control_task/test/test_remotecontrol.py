import unittest
from unittest.mock import patch
import pytest
import rclpy
from geometry_msgs.msg import Twist
from src.remote_control_task.remote_control_task.remote_control import remote_control


class remote_controlLocalVelocityTest(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.test_node = remote_control()

    def tearDown(self):
        self.test_node.destroy_node()

    def test_localVelocity_forward(self):
        with patch.object(self.test_node, 'getKey', return_value="w"):
            rclpy.spin_once(self.test_node)
        self.assertEqual(self.test_node.getLinearVelocity(), 0.01)
        self.assertEqual(self.test_node.getAngularVelocity(), 0.0)

    def test_localVelocity_backward(self):
        with patch.object(self.test_node, 'getKey', return_value="s"):
            rclpy.spin_once(self.test_node)
        self.assertEqual(self.test_node.getLinearVelocity(), -0.01)
        self.assertEqual(self.test_node.getAngularVelocity(), 0.0)

    def test_localVelocity_left(self):
        with patch.object(self.test_node, 'getKey', return_value="a"):
            rclpy.spin_once(self.test_node)
        self.assertEqual(self.test_node.getAngularVelocity(), 0.1)
        self.assertEqual(self.test_node.getLinearVelocity(), 0.0)

    def test_localVelocity_right(self):
        with patch.object(self.test_node, 'getKey', return_value="d"):
            rclpy.spin_once(self.test_node)
        self.assertEqual(self.test_node.getAngularVelocity(), -0.1)
        self.assertEqual(self.test_node.getLinearVelocity(), 0.0)

    def test_localVelocity_emergencyStop(self):
        self.test_localVelocity_forward()
        self.test_localVelocity_left()
        self.assertEqual(self.test_node.getLinearVelocity(), 0.01)

        with patch.object(self.test_node, 'getKey', return_value="e"):
            rclpy.spin_once(self.test_node)
        self.assertEqual(self.test_node.getLinearVelocity(), 0.0)
        self.assertEqual(self.test_node.getAngularVelocity(), 0.0)


class remote_controlPublishedVelocityTest(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.test_node = remote_control()
        self.sub_node = rclpy.create_node('sub_remote_control')
        self.received_msgs = []
        self.sub_node.create_subscription(
            Twist,
            'cmd_vel',
            lambda msg: self.received_msgs.append(msg),
            10
        )

    def tearDown(self):
        self.test_node.destroy_node()
        self.sub_node.destroy_node()

    def check_Twist_message(self, index, linear, angular):
        self.assertEqual(self.received_msgs[index].linear.x, linear)
        self.assertEqual(self.received_msgs[index].linear.y, 0.0)
        self.assertEqual(self.received_msgs[index].linear.z, 0.0)
        self.assertEqual(self.received_msgs[index].angular.x, 0.0)
        self.assertEqual(self.received_msgs[index].angular.y, 0.0)
        self.assertEqual(self.received_msgs[index].angular.z, angular)

    def test_publishedVelocity_forward(self):
        with patch.object(self.test_node, "getKey", return_value="w"):
            rclpy.spin_once(self.test_node)
            rclpy.spin_once(self.sub_node)
        self.assertEqual(len(self.received_msgs), 1)
        self.check_Twist_message(0, 0.01, 0.0)

    def test_publishedVelocity_backward(self):
        with patch.object(self.test_node, "getKey", return_value="s"):
            rclpy.spin_once(self.test_node)
            rclpy.spin_once(self.sub_node)
        self.assertEqual(len(self.received_msgs), 1)
        self.check_Twist_message(0, -0.01, 0.0)

    def test_publishedVelocity_left(self):
        with patch.object(self.test_node, "getKey", return_value="a"):
            rclpy.spin_once(self.test_node)
            rclpy.spin_once(self.sub_node)
        self.assertEqual(len(self.received_msgs), 1)
        self.check_Twist_message(0, 0.0, 0.1)

    def test_publishedVelocity_right(self):
        with patch.object(self.test_node, "getKey", return_value="d"):
            rclpy.spin_once(self.test_node)
            rclpy.spin_once(self.sub_node)
        self.assertEqual(len(self.received_msgs), 1)
        self.check_Twist_message(0, 0.0, -0.1)

    def test_publishedEmergencyStop(self):
        self.test_publishedVelocity_forward()

        with patch.object(self.test_node, "getKey", return_value="d"):
            rclpy.spin_once(self.test_node)
            rclpy.spin_once(self.sub_node)
        self.assertEqual(len(self.received_msgs), 2)
        self.check_Twist_message(1, 0.01, -0.1)

        with patch.object(self.test_node, "getKey", return_value="e"):
            rclpy.spin_once(self.test_node)
            rclpy.spin_once(self.sub_node)
        self.assertEqual(len(self.received_msgs), 3)
        self.check_Twist_message(2, 0.0, 0.0)


if __name__ == '__main__':
    unittest.main()
