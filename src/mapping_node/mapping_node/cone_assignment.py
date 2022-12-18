import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data, QoSProfile


class ConeAssignment(Node):

    def __init__(self):
        super().__init__('coneAssignment')
        self.sub = self.create_subscription(Float32MultiArray, 'coneAssignment', self.callback, 10)
        self.qos = QoSProfile(depth=10)
        self.latest_lidar_info = None
        self.sub_lidar = self.create_subscription(LaserScan, 'scan', self.setLatestLidarInfo,
                                                  qos_profile=qos_profile_sensor_data)

    def setLatestLidarInfo(self, data):
        self.latest_lidar_info = data.ranges

    def callback(self, detections):
        bBox_coordinates = detections.data.tolist()
        try:
            if self.latest_lidar_info is not None and bBox_coordinates is not None:
                result = self.map_lidar_to_image(bBox_coordinates, self.latest_lidar_info)
                # result = [Winkel der Mitte von bBox, boxmitte in pixel, distance, color]
                self.get_logger().info(str(result))
        except Exception as err:
            self.get_logger().info(str(err))

    # takes a lidar range list and returns a tuple with:
    # [0] list of angle ranges of found objects (cam angles)
    # [1] list of distances of these objects
    def get_angles(self, ranges):
        lidar_object_angles = []
        lidar_object_distances = []
        ranges = ranges[::-1]
        angle = 0
        cone_distance = []
        for i in range(149, 212):
            if ranges[i] > 0:  # object detected
                if len(cone_distance) > 0:  # check if found an object before
                    if abs(cone_distance[-1] - ranges[i]) < 0.1:  # check for same object
                        angle += 1
                        cone_distance.append(ranges[i])
                    else:  # other object found
                        lidar_object_angles.append([i - angle - 149, i - 149])
                        lidar_object_distances.append(sum(cone_distance) / len(cone_distance))
                        angle = 1
                        cone_distance = [ranges[i]]
                else:  # new object found
                    angle += 1
                    cone_distance.append(ranges[i])

            elif angle > 0:  # no object detected
                lidar_object_angles.append([i - angle, i - 149])
                lidar_object_distances.append(sum(cone_distance) / len(cone_distance))
                angle = 0
                cone_distance.clear()
        lidar_object_angles.append([i - angle - 149, i - 149])
        lidar_object_distances.append(sum(cone_distance) / len(cone_distance))
        return lidar_object_angles, lidar_object_distances

    # takes tensor output and returns a list with entries: [box start pixel, box end pixel, color]
    def extract_bounding_ranges(self, cone_cords_list):
        cone_pixel_list = []
        for i in range(0, len(cone_cords_list), 3):
            cone_pixel_list.append([cone_cords_list[i], cone_cords_list[i+1], cone_cords_list[i+2]])
        return cone_pixel_list

    # takes the angle list of get_angles and converts the angles to pixel values
    def convert_lidar_angles_to_pixels(self, angle_list):
        ratio = 640 / 62
        lidar_object_pixels = []
        for lidar_object in angle_list:
            lidar_object_pixels.append([lidar_object[0] * ratio, lidar_object[1] * ratio])
        return lidar_object_pixels

    # input: list of cone border points, list of lidar object pixels, lidar object angles and lidar object distances
    # assigns each cone border a matching object from the lidar data
    # outputs a list with entries [angle of cone, middle of bounding box, distance, color]
    def assign_matching_bounding_box(self, cone_border_pixels, lidar_object_pixels, lidar_object_angles,
                                     lidar_object_distances):
        angle_pixel_ratio = 640 / 62
        assignments = []
        for cone in cone_border_pixels:
            for lidar_object in lidar_object_pixels:
                index = lidar_object_pixels.index(lidar_object)
                if (cone[0] < lidar_object[0]) and (cone[1] > lidar_object[1]):
                    assignments.append(
                        [(lidar_object_angles[index][0] + lidar_object_angles[index][1]) / 2, (cone[0] + cone[1]) / 2,
                         lidar_object_distances[index], cone[2]])
        return assignments

    # input: tensor output, lidar data 'ranges' combines all functions above and returns a list with entries (angle
    # of cone, position of cone in the image in pixels, distance in meter, color label)
    def map_lidar_to_image(self, bounding_boxes, ranges):
        cone_pixel_list = []  # pixel ranges of the detected cones (A.x - B.x)
        lidar_object_angles = []  # angle ranges of detected objects
        lidar_object_pixels = []  #
        lidar_object_distances = []  # averaged distance of the detected objects
        assignments = []  # result list
        cone_pixel_list = self.extract_bounding_ranges(bounding_boxes)
        lidar_object_angles, lidar_object_distances = self.get_angles(ranges)
        lidar_object_pixels = self.convert_lidar_angles_to_pixels(lidar_object_angles)
        assignments = self.assign_matching_bounding_box(cone_pixel_list, lidar_object_pixels, lidar_object_angles,
                                                        lidar_object_distances)
        return assignments


def main(args=None):
    rclpy.init(args=args)

    node = ConeAssignment()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
