import sys
import matplotlib
import matplotlib.pyplot as plt
import time
import rclpy
from rclpy.node import Node
matplotlib.use('Qt5Agg')
from PyQt5 import QtCore, QtWidgets
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg
from matplotlib.figure import Figure
from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import LaserScan
import math
from rclpy.qos import qos_profile_sensor_data


class Worker(QRunnable):
    def __init__(self, fn, *args, **kwargs):
        super(Worker, self).__init__()
        self.fn = fn
        self.args = args
        self.kwargs = kwargs

    @pyqtSlot()
    def run(self):
        self.fn(*self.args, **self.kwargs)


class LidarPrinter(Node):
    def __init__(self):
        super().__init__('lidar_subscriber')
        self.create_subscription(Float64MultiArray, 'lid_print', self.listener_test_callback, 10)
        self.create_subscription(Float64MultiArray, 'cone_print', self.listener_cone_callback, 10)
        self.create_subscription(LaserScan, 'scan', self.listener_callback, qos_profile=qos_profile_sensor_data)
        self.interface = None

    def listener_test_callback(self, msg):
        print('msg')
        print(list(msg.data))
        self.interface.update_lidar_ranges(msg.data)

    # TODO: Test this
    def listener_callback(self, msg):
        print('LIDAR ranges')
        print(list(msg.ranges))
        self.interface.update_lidar_ranges(msg.ranges)
        #print('incoming')

    def listener_cone_callback(self, msg):
        print('Cone ranges')
        print(list(msg.data))
        self.interface.update_cone_ranges(msg.data[0], msg.data[1], msg.data[2])

    # TODO: Put cones into LIDAR
    def set_interface(self, interface):
        self.interface = interface

    def start(self):
        print('Lidar node started')
        rclpy.spin(self)
        self.destroy_node()
        rclpy.shutdown()


class LidarCanvas(FigureCanvasQTAgg):
    def __init__(self, parent=None, width=200, height=160, dpi=100):
        fig = Figure(figsize=(width, height), dpi=dpi)
        self.axes = fig.add_subplot(projection='polar')
        self.axes.set_theta_zero_location('W', offset=90)
        self.axes.set_rorigin(-1)
        self.axes.set_thetamin(220)
        self.axes.set_thetamax(140)
        self.axes.set_ylim([0, 3.5])
        super(LidarCanvas, self).__init__(fig)


# TODO: Add map and photo
class Main(QWidget):
    def __init__(self, node=None, angle_min=0.0, angle_inc=1, *args, **kwargs):
        super().__init__()

        self.threadpool = QThreadPool()

        self.lidar_node = node

        # Create the maptlotlib FigureCanvas object,
        # which defines a single set of axes as self.axes.
        self.lidar_canvas = LidarCanvas(self, width=200, height=160, dpi=100)
        self.xdata = [(angle_min + angle_inc * i) for i in range(0, 360)]
        self.ydata = [0]*360

        grid = QGridLayout()
        self.setLayout(grid)

        lidar_button = QtWidgets.QPushButton("LIDAR")
        lidar_button.clicked.connect(self.scan_lidar)
        grid.addWidget(lidar_button, 1, 1)
        grid.addWidget(self.lidar_canvas, 1, 2)

        self._plot_lidar_ranges_ref = None
        self.update_lidar_ranges(self.ydata)

        self._plot_cone_refs = [None, None, None]
        self.update_cone_range([180], [2.5], 0)
        self.update_cone_range([170], [2.5], 1)
        self.update_cone_range([190], [2.5], 2)

        self.show()

    def scan_lidar(self, args=None):
        self.lidar_node.set_interface(self)
        worker = Worker(self.lidar_node.start)
        self.threadpool.start(worker)

    def update_lidar_ranges(self, ranges, angle_min=0.0, angle_inc=2 * math.pi / 360):
        # print([(angle_min + angle_inc * i) for i in range(0, 360)])

        self.xdata = [(angle_min + angle_inc * i) for i in range(0, 360)]
        self.ydata = ranges

        if self._plot_lidar_ranges_ref is None:
            plot_refs = self.lidar_canvas.axes.plot(self.xdata, ranges, 'o', ms=3)
            self._plot_lidar_ranges_ref = plot_refs[0]
        else:
            self._plot_lidar_ranges_ref.set_ydata(ranges)
            self.lidar_canvas.draw_idle()

    # TODO:
    def update_cone_range(self, angle, distance, cone):
        # Takes a list of angles and a list of distances and plots them
        if cone == 0:
            color = 'orange'
        elif cone == 1:
            color = 'b'
        else:
            color = 'y'

        for x in range(len(angle)):
            angle[x] = angle[x] / 360 * 2 * math.pi

        if self._plot_cone_refs[cone] is None:
            plot_refs = self.lidar_canvas.axes.plot(angle, distance, 'o', c=color, ms=8)
            self._plot_cone_refs[cone] = plot_refs[0]
        else:
            self._plot_cone_refs[cone].set_xdata(angle)
            self._plot_cone_refs[cone].set_ydata(distance)
            self.lidar_canvas.draw_idle()




if __name__ == "__main__":
    args = None
    rclpy.init(args=args)
    lidar_node = LidarPrinter()

    app = QtWidgets.QApplication(sys.argv)
    w = Main(node=lidar_node)
    app.exec_()
