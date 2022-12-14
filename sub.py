import rclpy
from rclpy.qos import qos_profile_sensor_data, QoSProfile
from sensor_msgs.msg import LaserScan


def callback(data):
    print(data)


rclpy.init()
qos = QoSProfile(depth=10)
sub_node = rclpy.create_node("sub")
sub = sub_node.create_subscription(LaserScan, 'scan', lambda msg: callback(msg), qos_profile=qos_profile_sensor_data)
rclpy.spin(sub_node)
sub_node.destroy_node()
rclpy.shutdown()
