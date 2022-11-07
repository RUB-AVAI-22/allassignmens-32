import sys
import os
import rclpy
from rclpy.node import Node
from PyQt5.QtWidgets import *
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import threading
from colorama import Fore
from colorama import Style

class DisNode(Node):
    def __init__(self):
        super().__init__('Disnode')
        self.counter=0
        self.current_frame = None
        self.br = CvBridge()
        self.subscription = self.create_subscription(Image,'STORAGE',self.subscribe_callback,10)
        self.subscription

    def subscribe_callback(self, data):
        try:
            self.current_frame = self.br.imgmsg_to_cv2(data)
            self.get_logger().info(f'{Fore.GREEN}Receiving video frame_{self.counter} from Topic STORAGE{Style.RESET_ALL}')
            cv2.imshow("camera", self.current_frame)
            path = 'src/imcappkg/imcappkg/Images'
            file_name = 'img_{}.jpg'.format(self.counter)
            try:
                a = cv2.imwrite(os.path.join(path , file_name), self.current_frame)
                if a :
                    print(f'\t\t\t\t\t {Fore.MAGENTA}just saved img_{self.counter} into folder{Style.RESET_ALL}')
                else:
                    print(f"{Fore.RED}!!!!!!!!!!!!!!!!!! No images is saved !!!!!!!!!!!!!!!!!{Style.RESET_ALL}")
            except Exception as e:
                print(e)
            self.counter = self.counter + 1
            if cv2.waitKey(1) & 0xFF == ord('q'):
                cv2.destroyAllWindows()
                sys.exit()
        except Exception as e:
            print(e)

class mainwin(QWidget):
    def __init__(self, dnode):
        super(mainwin, self).__init__()
        self.dNode = dnode
        self.initWin()
        self.t = None

    def initWin(self):
        self.setGeometry(300, 300, 150, 150)
        frame_w = self.frameGeometry()
        wincenter = QDesktopWidget().availableGeometry().center()
        frame_w.moveCenter(wincenter)
        self.setWindowTitle(" Camera Sensor ")
        layout = QGridLayout()
        self.setLayout(layout)
        positions = [(0, 0), (0, 1),(1,0), (1,1) , (1,2)]
        self.frec_lbl = QLabel(self)
        self.frec_lbl.setText("Frecuency: ")
        self.frec_lbl.setStyleSheet("font:bold 15px ; color: black")
        self.frec_lbl.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        layout.addWidget(self.frec_lbl, *positions[0])
        self.frec_input = QLineEdit(self)
        self.frec_input.setFixedSize(300, 35)
        self.frec_input.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        layout.addWidget(self.frec_input, *positions[1])
        self.btn_cap = QPushButton(self)
        self.btn_cap.setText("Display")
        self.btn_cap.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.btn_cap.setStyleSheet("background-color: sienna; font:15px;")
        layout.addWidget(self.btn_cap, *positions[2])
        self.btn_cap.clicked.connect(self.onclicked_display)#self.dNode.subscribe_callback)

        self.btn_stop = QPushButton(self)
        self.btn_stop.setText("Stop")
        self.btn_stop.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.btn_stop.setStyleSheet("background-color: sienna; font:15px;")
        layout.addWidget(self.btn_stop, *positions[3])
        self.btn_stop.clicked.connect(self.onclicked_stop)  # self.dNode.subscribe_callback)

        self.btn_setfrec = QPushButton(self)
        self.btn_setfrec.setText("Change frecuency")
        self.btn_setfrec.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.btn_setfrec.setStyleSheet("background-color: sienna; font:15px;")
        layout.addWidget(self.btn_setfrec, *positions[4])
        self.btn_setfrec.clicked.connect(self.onclicked_setfrec)

    def onclicked_display(self):
        try:
            self.btn_cap.setEnabled(False)
            self.t = threading.Thread(target = rclpy.spin(self.dNode))
            self.t.start()
        except Exception as e:
            print(e)

    def onclicked_stop(self):
        try:
            self.btn_cap.setEnabled(True)
            #self.t.join()
            cv2.destroyAllWindows()
            sys.exit()
        except Exception as e:
            print(e)

    def onclicked_setfrec(self):
        if self.frec_input.text():
            try:
                stream1 = os.popen('ros2 param set /Camnode fps {}'.format(self.frec_input.text()))
                stream2= os.popen('ros2 param set /Procnode PRtoSTORAGE {}'.format(self.frec_input.text()))
                output1 = stream1.read()
                output2 = stream2.read()
                QMessageBox.critical(self,'info', output1)
                QMessageBox.critical(self,'info',output2)
            except Exception as e:
                print(e)
        else:
            QMessageBox.critical(self,'info', 'Enter a Frecuency first !')


def main(args=None):
    rclpy.init(args=args)
    dis_node = DisNode()
    app = QApplication(sys.argv)
    win = mainwin(dis_node)
    win.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()