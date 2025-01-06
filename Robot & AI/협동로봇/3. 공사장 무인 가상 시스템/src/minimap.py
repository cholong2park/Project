import sys
import rclpy
import threading
from rclpy.node import Node
from sensor_msgs.msg import Image
from PySide2.QtCore import *
from PySide2.QtWidgets import *
from PySide2.QtGui import QPixmap, QImage
from geometry_msgs.msg import PoseWithCovarianceStamped, Point, Quaternion
from std_msgs.msg import String
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import SetInitialPose
from rclpy.action import ActionClient

import yaml
from PIL import Image, ImageDraw



class NODE(QThread, Node):
    signal = Signal(list)
    alert = Signal(str)
    def __init__(self, node_name='ros_subscriber_node'):
        QThread.__init__(self)
        Node.__init__(self, node_name)

        self.amcl_pose_x = 0
        self.amcl_pose_y = 0

        self.dot_size = 2

        self.sub_cmd_vel = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.subscription_callback, 10)
        self.alert_sub = self.create_subscription(String, 'alert', self.alert_callback, 10)
        self._action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.set_initial_pose_service_client = self.create_client(
            SetInitialPose,
            '/set_initial_pose'
            )
        while not self.set_initial_pose_service_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service /set_initial_pose not available, waiting again...')
        self.init_pose = [0.0, 0.0, 0.0, 1.0]
        self.set_initial_pose(*self.init_pose)

    def alert_callback(self, msg):
        self.alert.emit(msg.data)

    def subscription_callback(self, msg):
        self.amcl_pose_x = msg.pose.pose.position.x
        self.amcl_pose_y = msg.pose.pose.position.y
        self.get_logger().info(f'Received amcl_pose x: {self.amcl_pose_x}, y: {self.amcl_pose_y}')

        self.signal.emit([self.amcl_pose_x, self.amcl_pose_y])
        self.get_logger().info(f'emit signal')

    def run(self):
        rclpy.spin(self)

    def set_initial_pose(self, x,y,z,w):
        req = SetInitialPose.Request()
        req.pose.header.frame_id = 'map'
        req.pose.pose.pose.position = Point(x=x, y=y, z=0.0)
        req.pose.pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=z, w=w)
        req.pose.pose.covariance = [0.1, 0.0, 0.0, 0.0, 0.0, 0.1,
                              0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                              0.0, 0.0, 0.1, 0.0, 0.0, 0.0,
                              0.0, 0.0, 0.0, 0.01, 0.0, 0.0,
                              0.0, 0.0, 0.0, 0.0, 0.01, 0.0,
                              0.0, 0.0, 0.0, 0.0, 0.0, 0.01]
        future = self.set_initial_pose_service_client.call_async(req)
        self.get_logger().info("[INFO] Initial pose set successfully")
        return future.result()

    def send_goal(self, x, y, theta):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.z = theta
        goal_msg.pose.pose.orientation.w = 1.0
        self._action_client.wait_for_server()
        self.get_logger().info(f'Sending goal to: x={x}, y={y}, theta={theta}')
        self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        ).add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        # self.get_logger().info(f"Current progress: {feedback.current_pose}")

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return
        self.get_logger().info('Goal accepted :)')
        goal_handle.get_result_async().add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result}')

class GUI(QMainWindow):
    def __init__(self, ros_thread):
        super().__init__()
        self.ros_thread = ros_thread
        self.ros_thread.signal.connect(self.process_signal)
        self.ros_thread.alert.connect(self.showAlert)

        self.resize = (300,300)
        self.dot_size=2

        image = Image.open('map.pgm')
        self.width, self.height = image.size
        self.image_rgb = image.convert('RGB')

        with open('map.yaml', 'r') as file:
            data = yaml.safe_load(file)

        self.resolution = data['resolution']
        self.map_x= -data['origin'][0]
        self.map_y= +data['origin'][1] +self.height *self.resolution

        self.x = self.map_x
        self.y = self.map_y

        self.setupUi()

    def setupUi(self):
        self.window = QMainWindow()
        if not self.window.objectName():
            self.window.setObjectName(u"MainWindow")
        self.window.setObjectName("MainWindow")
        self.window.resize(340, 400)

        self.centralwidget = QWidget(self.window)
        self.centralwidget.setObjectName("centralwidget")

        # Label 설정
        self.label = QLabel(self.centralwidget)
        self.label.setObjectName("label")
        self.label.setGeometry(20, 20, 320, 320)

        # 시작 버튼 추가
        self.start_button = QPushButton(self.centralwidget)
        self.start_button.setObjectName("start_button")
        self.start_button.setGeometry(120, 350, 100, 30)  # 위치와 크기 설정
        self.start_button.setText("시작")

        # 버튼 클릭 이벤트 연결
        self.start_button.clicked.connect(self.on_start_button_clicked)

        self.window.setCentralWidget(self.centralwidget)

    def on_start_button_clicked(self):
        # 시작 버튼 클릭 시 실행될 동작
        self.ros_thread.send_goal(4.5,1.0,0.0)

    def showAlert(self, message):
        """Show an alert message box."""
        alert = QMessageBox(self.window)
        alert.setWindowTitle("Alert")
        alert.setText(message)
        alert.setIcon(QMessageBox.Information)
        alert.exec_()
        
    def process_signal(self, message):
        odom_x = message[0]
        odom_y = message[1]
        self.x = self.map_x +odom_x
        self.y = self.map_y -odom_y

        print(f"x={self.x}, y={self.y}")
        self.input_image()

    def input_image(self):
        image_copy = self.image_rgb.copy()
        draw = ImageDraw.Draw(image_copy)
        draw.ellipse((
                self.x /self.resolution -self.dot_size, 
                self.y /self.resolution -self.dot_size, 
                self.x /self.resolution +self.dot_size, 
                self.y /self.resolution +self.dot_size), 
            fill='red')

        image_rotated = image_copy.rotate(90, expand=True)
        image_resized = image_rotated.resize(self.resize)
        pil_image = image_resized.convert('RGBA')  # QImage는 RGBA 포맷을 사용
        data = pil_image.tobytes("raw", "RGBA")
        qimage = QImage(data, *self.resize, QImage.Format_RGBA8888)
        pixmap = QPixmap.fromImage(qimage)
        self.label.setPixmap(pixmap)


def main():
    rclpy.init()
    node = NODE()
    ros_thread = threading.Thread(target=lambda : rclpy.spin(node), daemon=True)
    ros_thread.start()

    app = QApplication(sys.argv)
    gui = GUI(node)
    gui.window.show()
    
    try:
        sys.exit(app.exec_())
    finally:
        node.destroy_node()
        rclpy.shutdown()
    
if __name__ == '__main__':
    main()

    