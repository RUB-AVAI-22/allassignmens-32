import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data, QoSProfile
import numpy as np
from matplotlib import pyplot as plt


class ConeAssignment(Node):

    def __init__(self):
        super().__init__('coneAssignment')
        self.sub = self.create_subscription(Float32MultiArray, 'coneAssignment', self.callback, 10)
        self.qos = QoSProfile(depth=10)
        self.latest_lidar_info = None
        self.assigned_cones = []
        self.sub_lidar = self.create_subscription(LaserScan, 'scan', self.setLatestLidarInfo,
                                                  qos_profile=qos_profile_sensor_data)
        _, self.map = plt.subplots()
        self.bot_x = 0
        self.bot_y = 0
        self.bot_angle = 0
        self.cache_x_coordinates = []
        self.cache_y_coordinates = []
        self.cache_colors = ['black']
        self.cache_conf = [1.0]

    def setLatestLidarInfo(self, data):
        self.latest_lidar_info = data.ranges

    def callback(self, detections):
        bBox_coordinates = detections.data.tolist()
        bBox_list = self.extract_bounding_ranges(bBox_coordinates)  # x1, y1, x2, y2, conf, label
        try:
            if self.latest_lidar_info is not None and bBox_coordinates is not None:
                self.assign_cones(bBox_list)
                self.draw_map()
        except Exception as err:
            self.get_logger().info(str(err))

    def assign_cones(self, detections):
        cones = []
        for cone in detections:
            bBox_start, bBox_end = self.calculate_lidar_range(cone)
            difference = bBox_end - bBox_start
            offset = 3.5
            bBox_start += offset
            bBox_end += offset
            shift = difference * 1 / 6  # might be that we can delete shift and replace it with end

            cone_distances = self.latest_lidar_info[round(bBox_start + shift):round(bBox_end - shift)]
            cone_distances = list(
                filter(lambda x: 0 < x < 3, cone_distances))  # we filter out ranges above 3m and of 0m/infinite
            if len(cone_distances) < 2:
                continue
            else:
                distance = np.median(cone_distances)
                angle = (bBox_start + bBox_end) / 2.0
                print("Detected: ", angle, distance, cone[4], cone[5])
                cones.append([angle, distance, cone[4], cone[5]])  # angle, distance, conf, color
        self.assigned_cones = cones

    def draw_map(self):
        self.map.cla()  # clear current axes

        #  draw robot
        self.cache_x_coordinates[0] = self.bot_x
        self.cache_y_coordinates[0] = self.bot_y

        #  draw cones
        for cone in self.assigned_cones:
            x, y = self.get_map_coords(cone[0], cone[1])
            self.cache_x_coordinates.append(x + self.bot_x)
            self.cache_y_coordinates.append(y + self.bot_y)
            self.cache_conf.append(cone[2])
            if cone[3] == 0:
                self.cache_colors.append('blue')
            elif cone[3] == 1:
                self.cache_colors.append('orange')
            elif cone[3] == 2:
                self.cache_colors.append('yellow')

        self.map.scatter(self.cache_x_coordinates, self.cache_y_coordinates,
                         color=self.cache_colors, alpha=self.cache_conf)
        plt.pause(0.1)

    def get_map_coords(self, angle, distance):
        rad = angle * (np.pi / 180)
        x = distance * np.cos(rad)
        y = distance * np.sin(-rad)
        return x, y

    def calculate_lidar_range(self, cone):
        fov = 62.2
        left = 180 - (fov / 2)
        pixels = 640
        start = left + cone[0] * fov / pixels
        end = left + cone[2] * fov / pixels

        return start, end

    # takes tensor output and returns a list with entries: [box start pixel, box end pixel, color]
    def extract_bounding_ranges(self, cone_cords_list):
        cone_pixel_list = []
        for i in range(0, len(cone_cords_list), 6):
            cone_pixel_list.append(
                [cone_cords_list[i], cone_cords_list[i + 1], cone_cords_list[i + 2], cone_cords_list[i + 3],
                 cone_cords_list[i + 4], cone_cords_list[i + 5]])
        return cone_pixel_list


def main(args=None):
    rclpy.init(args=args)

    node = ConeAssignment()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
