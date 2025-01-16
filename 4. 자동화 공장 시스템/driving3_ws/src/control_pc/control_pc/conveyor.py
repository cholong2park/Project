import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String, Int32
import serial


class StepControllerNode(Node):
    def __init__(self):
        super().__init__('step_controller')
        # 구독자 설정
        self.publisher = self.create_publisher(String, 'conveyor_status', 10)
        self.subscription = self.create_subscription(
            Int32,
            'step',
            self.listener_callback,
            10
        )
        self.done_sub = self.create_subscription(Int32, 'done', self.done_callback, 10)

        self.subscription  # prevent unused variable warning

        # 시리얼 설정
        self.serial_port = '/dev/ttyACM0'  # 아두이노와 연결된 포트
        self.baud_rate = 115200
        self.arduino = None
        self.connected = False

        # Arduino 연결 시도
        self.connect_to_arduino()

        # 타이머 초기화
        self.timer = None
        self.step_active = False  # 상태 추적
        self.timer = self.create_timer(1, self.check_conveyor_status)

        # 상태 추적 변수 초기화
        self.current_status = None  # 현재 상태
        self.previous_status = None  # 이전 상태

    def done_callback(self, msg):
        if msg.data==0:
            if self.step_active:  # 활성화 상태라면 타이머 중지
                self.step_active = False
                if self.timer is not None:
                    self.timer.cancel()
                    self.timer = None
                step_count = 10000 # 보낼 스텝 수
                self.arduino.write(f"{step_count}\n".encode('utf-8'))
                self.get_logger().info("Step sending stop.")
    def connect_to_arduino(self):
        try:
            self.arduino = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
            self.connected = True
            self.get_logger().info(f"Connected to Arduino on {self.serial_port}")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to connect to Arduino: {e}")
            if self.connected:
                self.current_status = "ERROR"
                self.publish_status(self.current_status)
            self.connected = False

    def listener_callback(self, msg):
        if msg.data==0:  # step = 1
            if not self.step_active:  # 이미 활성화 상태가 아니면 타이머 시작
                self.step_active = True
                self.timer = self.create_timer(0.1, self.send_steps_to_arduino)
                self.get_logger().info("Step sending started.")
        elif msg.data==1:  # step = 0
            if self.step_active:  # 활성화 상태라면 타이머 중지
                self.step_active = False
                if self.timer is not None:
                    self.timer.cancel()
                    self.timer = None
                self.get_logger().info("Step sending now stopped.")
    def send_steps_to_arduino(self):
        if self.arduino is not None and self.step_active:
            step_count = 100 # 보낼 스텝 수
            self.arduino.write(f"{step_count}\n".encode('utf-8'))
        else:
            self.get_logger().error("Arduino is not connected or step is inactive.")

    def check_conveyor_status(self):
        # Arduino 연결 상태 확인
        if self.arduino is None or not self.connected:
            self.connect_to_arduino()
            return

        try:
            if self.arduino.in_waiting > 0:
                data = self.arduino.read(self.arduino.in_waiting).decode('utf-8').strip()
                if data:
                    for char in data:
                        if char == '.':
                            self.current_status = "STOP"
                        elif char == '_':
                            self.current_status = "PLAY"
                        else:
                            self.current_status = "ERROR"

                        if self.current_status != self.previous_status:
                            self.previous_status = self.current_status
                            self.publish_status(self.current_status)
            else:
                self.get_logger().info("No data available from Arduino.")
        except (serial.SerialException, OSError) as e:
            self.get_logger().error(f"Error during serial communication: {e}")
            if self.connected:
                self.current_status = "ERROR"
                self.publish_status(self.current_status)
            self.connected = False

    def publish_status(self, status):
        msg = String()
        msg.data = status
        self.publisher.publish(msg)
        self.get_logger().info(f"Conveyor status: {status}")

def main(args=None):
    rclpy.init(args=args)
    node = StepControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.timer is not None:
            node.timer.cancel()
        if node.arduino is not None:
            node.arduino.close()
        node.destroy_node()
        rclpy.shutdown()