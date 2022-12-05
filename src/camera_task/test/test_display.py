import unittest
from unittest.mock import patch
import rclpy
import sys
import os
import cv2
from std_msgs.msg import String
import pytest

sys.path.append(os.path.dirname(__file__) + "/../camera_task")
from image_display import DisplayNode


class DisplayNodeCommandTest(unittest.TestCase):

    tests_timeout = 300

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        with patch("cv2.namedWindow", return_value=None):
            with patch("pathlib.Path.mkdir", return_value=None):
                self.test_node = DisplayNode()
        self.captured_cmds = []
        self.sub_node = rclpy.create_node('sub_display')
        self.sub_node.create_subscription(
            String,
            'externalCommandStream',
            lambda cmd: self.captured_cmds.append(cmd),
            10
        )

    def tearDown(self):
        self.test_node.destroy_node()
        self.sub_node.destroy_node()

    @pytest.mark.timeout(tests_timeout)
    def test_requestPicture(self):
        commands = ["f", "x"]
        with patch("builtins.input", side_effect=commands):
            self.test_node.handle_keyboard()
            rclpy.spin_once(self.sub_node)
        self.assertEqual(len(self.captured_cmds), 1)
        self.assertEqual(self.captured_cmds[0].data, "f")

    @pytest.mark.timeout(tests_timeout)
    def test_changeFrequencyPositiveInteger(self):
        commands = [5, "x"]
        with patch("builtins.input", side_effect=commands):
            self.test_node.handle_keyboard()
            rclpy.spin_once(self.sub_node)
        self.assertEqual(len(self.captured_cmds), 1)
        self.assertEqual(self.captured_cmds[0].data, "5")

    @pytest.mark.timeout(tests_timeout)
    def test_changeFrequencyNegativeInteger(self):
        commands = [-5, "x"]
        with patch("builtins.input", side_effect=commands):
            self.test_node.handle_keyboard()
            rclpy.spin_once(self.sub_node)
        self.assertEqual(len(self.captured_cmds), 1)
        self.assertEqual(self.captured_cmds[0].data, "-5")

    @pytest.mark.timeout(tests_timeout)
    def test_changeFrequencyPositiveFloat(self):
        commands = [5.23, "x"]
        with patch("builtins.input", side_effect=commands):
            self.test_node.handle_keyboard()
            rclpy.spin_once(self.sub_node)
        self.assertEqual(len(self.captured_cmds), 1)
        self.assertEqual(self.captured_cmds[0].data, "5.23")

    @pytest.mark.timeout(tests_timeout)
    def test_changeFrequencyNegativeFloat(self):
        commands = [-5.23, "x"]
        with patch("builtins.input", side_effect=commands):
            self.test_node.handle_keyboard()
            rclpy.spin_once(self.sub_node)
        self.assertEqual(len(self.captured_cmds), 1)
        self.assertEqual(self.captured_cmds[0].data, "-5.23")

    @pytest.mark.timeout(tests_timeout)
    def test_changeFrequencyZero(self):
        commands = [0, "x"]
        with patch("builtins.input", side_effect=commands):
            self.test_node.handle_keyboard()
            rclpy.spin_once(self.sub_node)
        self.assertEqual(len(self.captured_cmds), 1)
        self.assertEqual(self.captured_cmds[0].data, "0")

    @pytest.mark.timeout(tests_timeout)
    def test_invalidCommand(self):
        commands = ["j", "x"]
        with patch("builtins.input", side_effect=commands):
            self.test_node.handle_keyboard()
            rclpy.spin_once(self.sub_node)
        self.assertEqual(len(self.captured_cmds), 1)
        self.assertEqual(self.captured_cmds[0].data, "j")

    @pytest.mark.timeout(tests_timeout)
    def test_fuzzyInput(self):
        commands = ["asdjnefnweufbwednoaeewwwfbwafbil", "x"]
        with patch("builtins.input", side_effect=commands):
            self.test_node.handle_keyboard()
            rclpy.spin_once(self.sub_node)
        self.assertEqual(len(self.captured_cmds), 1)
        self.assertEqual(self.captured_cmds[0].data, "asdjnefnweufbwednoaeewwwfbwafbil")


if __name__ == '__main__':
    unittest.main()
