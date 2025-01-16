import sys
import sqlite3
from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QLabel, QLineEdit, QPushButton, 
    QMessageBox, QGridLayout, QGroupBox, QRadioButton, QHBoxLayout,
    QLCDNumber, QDialog, QTableWidget, QTableWidgetItem
)
from PyQt5.QtCore import QTimer, QDateTime, Qt, QThread, pyqtSignal
from PyQt5.QtGui import QImage, QPixmap
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool,String, Float32MultiArray, Int32MultiArray, Int32
from sensor_msgs.msg import CompressedImage
import numpy as np
import cv2
##############################################################
# 이메일에 필요한 모듈 및 패키지 설치
import smtplib
from email.mime.text import MIMEText
from email.mime.multipart import MIMEMultipart
##############################################################

# Database Setup
conn = sqlite3.connect('./src/factory.db')
cursor = conn.cursor()

#######################################################################################
# 이메일 보내는 함수
def send_email_gmail(sender_email, sender_password, recipient_email, subject, body):
    smtp_server = "smtp.gmail.com"
    smtp_port = 587  # TLS 포트

    # 이메일 구성
    message = MIMEMultipart()
    message["From"] = sender_email
    message["To"] = recipient_email
    message["Subject"] = subject
    message.attach(MIMEText(body, "plain"))

    try:
        # SMTP 연결 및 로그인
        server = smtplib.SMTP(smtp_server, smtp_port)
        server.starttls()  # TLS 암호화 시작
        server.login(sender_email, sender_password)
        
        # 이메일 전송
        server.sendmail(sender_email, recipient_email, message.as_string())
        print("관리자 이메일에 전송을 완료하였습니다!")
    except Exception as e:
        print(f"관리자 이메일에 전송을 실패하였습니다.: {e}")
    finally:
        server.quit()
#########################################################################################

# ROS 2 Node in a Separate QThread
class Ros2Thread(QThread):
    ros_message = pyqtSignal(str)  # Signal to communicate with PyQt5 GUI
    ros_image_signal=pyqtSignal(np.ndarray)
    ros_conveyor_status=pyqtSignal(str)
    marker_pose_signal = pyqtSignal(list)  # 추가된 marker pose 시그널

    def __init__(self):
        super().__init__()
        self.ros_node = Ros2Node(self.ros_image_signal,self.ros_conveyor_status, self.marker_pose_signal)

    def run(self):
        rclpy.spin(self.ros_node)

    def stop(self):
        self.ros_node.destroy_node()
        rclpy.shutdown()

class Ros2Node(Node):
    def __init__(self,ros_image_signal,ros_conveyor_status, marker_pose_signal):
        super().__init__('pyqt_ros2_node')
        self.image_signal=ros_image_signal
        self.conveyor_status=ros_conveyor_status
        self.img_sub=self.create_subscription(CompressedImage,'webcam_image/compressed',self.listner_callback,10)
        self.capture_pub = self.create_publisher(Bool, 'capture', 10)  # Bool 메시지 발행 준비
        # 좌표 토픽 구독 추가
        self.marker_pose_signal = marker_pose_signal  # 시그널 전달
        self.marker_pose_sub = self.create_subscription(
            Float32MultiArray, 'aruco_marker_pose', self.marker_pose_callback, 10
        )
        # Red Box, Blue Box 전달
        self.task_publisher = self.create_publisher(Int32MultiArray, 'task', 10)
        # Goal 위치 전달
        self.goal_publisher = self.create_publisher(Int32, 'goal', 10) 
        self.step_publisher = self.create_publisher(Int32, 'step', 10)
        self.conveyor_status_sub=self.create_subscription(String,'conveyor_status',self.status_callback,10)

    def publish_capture(self, value):
        msg = Bool()
        msg.data = value
        self.capture_pub.publish(msg)
        self.get_logger().info(f"Published: {value}")
###########################################################################################

    def status_callback(self,msg):
        self.get_logger().info(f"{msg.data}")
        self.conveyor_status.emit(msg.data)
        if msg.data == 'ERROR':
            ####################################################################################
            # 발신자 및 수신자 정보
            email = cursor.execute("SELECT admin_email FROM admin;").fetchone()[0]
            send_email_gmail(
                sender_email="tjdwocl85@gmail.com",  # Gmail 주소
                sender_password="uupugqxxaklcrmzz",  # 앱 비밀번호
                recipient_email= email,
                subject="컨베이어 벨트 에러",
                body="컨베이어 벨트에 에러가 발견되었습니다. 컨베이어 벨트를 확인해주십시오!"
            )
            ####################################################################################

    
    def publish_step(self, value):
        msg = Int32()
        msg.data = value
        self.step_publisher.publish(msg)
        self.get_logger().info(f"Published: {value}")
###########################################################################################
    def listner_callback(self,msg):
        np_arr=np.frombuffer(msg.data,np.uint8)
        frame=cv2.imdecode(np_arr,cv2.IMREAD_COLOR)

        if frame is not None:
            self.image_signal.emit(frame)  # Emit frame to PyQt GUI
        else:
            self.get_logger().warning('Failed to decode image')
    
    def marker_pose_callback(self, msg):
        """aruco_marker_pose 토픽 데이터 수신."""
        pose = list(msg.data)  # Float32MultiArray 데이터를 리스트로 변환
        self.get_logger().info(f"Received marker pose: {pose}")  # 디버깅용 로그
        self.marker_pose_signal.emit(pose)  # GUI로 데이터 전달

    def publish_box_data(self, red_box, blue_box):
        """red_box와 blue_box 데이터를 Int32MultiArray로 발행"""
        box_msg = Int32MultiArray()
        box_msg.data = [int(red_box), int(blue_box)]
        self.task_publisher.publish(box_msg)
        self.get_logger().info(f"Published Box Data: {box_msg.data}")

    def publish_goal_priority(self, goal_priority):
        """goal_priority 데이터를 Int32로 발행"""
        goal_msg = Int32()
        goal_msg.data = int(goal_priority)
        self.goal_publisher.publish(goal_msg)
        self.get_logger().info(f"Published Goal Priority: {goal_msg.data}")

# Login Window
class LoginWindow(QWidget):
    def __init__(self,ros_thread):
        super().__init__()
        self.setWindowTitle("로그인")
        self.setGeometry(300, 300, 400, 200)
        self.ros_thread = ros_thread
        
        layout = QVBoxLayout()
        
        self.id_input = QLineEdit(self)
        self.id_input.setPlaceholderText("아이디")
        self.password_input = QLineEdit(self)
        self.password_input.setPlaceholderText("패스워드")
        self.password_input.setEchoMode(QLineEdit.Password)
        
        self.login_button = QPushButton("로그인")
        self.login_button.clicked.connect(self.login)
        self.register_button = QPushButton("회원가입")
        self.register_button.clicked.connect(self.show_register)
        
        layout.addWidget(QLabel("로그인"))
        layout.addWidget(self.id_input)
        layout.addWidget(self.password_input)
        layout.addWidget(self.login_button)
        layout.addWidget(self.register_button)
        
        self.setLayout(layout)

    def login(self):
        user_id = self.id_input.text()
        password = self.password_input.text()
        
        cursor.execute("SELECT * FROM user WHERE user_name = ? AND password = ?", (user_id, password))
        user = cursor.fetchone()
        
        if user:
            QMessageBox.information(self, "로그인 성공", "로그인 되었습니다!")
            self.hide()
            self.main_window = MainWindow(user,self.ros_thread)
            self.main_window.show()
        else:
            QMessageBox.warning(self, "로그인 실패", "존재하지 않는 아이디나 패스워드입니다.\n다시 입력해주세요!")

    def show_register(self):
        self.register_window = RegisterWindow()
        self.register_window.show()


# Register Window
class RegisterWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("회원가입")
        self.setGeometry(300, 300, 400, 300)
        
        layout = QVBoxLayout()
        
        self.username_input = QLineEdit(self)
        self.username_input.setPlaceholderText("아이디")
        self.name_input = QLineEdit(self)
        self.name_input.setPlaceholderText("이름")
        self.email_input = QLineEdit(self)
        self.email_input.setPlaceholderText("이메일")
        self.password_input = QLineEdit(self)
        self.password_input.setPlaceholderText("패스워드")
        self.password_input.setEchoMode(QLineEdit.Password)
        
        self.register_button = QPushButton("회원가입")
        self.register_button.clicked.connect(self.register)
        
        layout.addWidget(QLabel("회원가입"))
        layout.addWidget(self.username_input)
        layout.addWidget(self.name_input)
        layout.addWidget(self.email_input)
        layout.addWidget(self.password_input)
        layout.addWidget(self.register_button)
        
        self.setLayout(layout)

    def register(self):
        username = self.username_input.text()
        name = self.name_input.text()
        email = self.email_input.text()
        password = self.password_input.text()
        
        try:
            cursor.execute("INSERT INTO user (user_name, name, email, password) VALUES (?, ?, ?, ?)", 
                           (username, name, email, password))
            conn.commit()
            QMessageBox.information(self, "회원가입 성공", "회원가입 되었습니다!")
            self.close()
        except sqlite3.IntegrityError:
            QMessageBox.warning(self, "회원가입 실패", "이미 존재하는 아이디나 이메일입니다!")


# Main Window
class MainWindow(QWidget):
    def __init__(self, user,ros_thread):
        super().__init__()
        self.user = user
        self.setWindowTitle("Task Management")
        self.setGeometry(300, 300, 600, 500)
        self.ros_thread=ros_thread
        self.ros_node=self.ros_thread.ros_node

        # Main Layout
        main_layout = QVBoxLayout()
        
        # Top Section: Admin Button
        top_layout = QHBoxLayout()
        self.admin_button = QPushButton("관리자 모드")
        self.admin_button.clicked.connect(self.open_admin_settings)
        top_layout.addStretch()
        top_layout.addWidget(self.admin_button)
        main_layout.addLayout(top_layout)

        # Task Section
        task_group = QGroupBox("Task")
        task_layout = QGridLayout()
        self.red_box_input = QLineEdit()
        self.blue_box_input = QLineEdit()
        self.goal_priority_1 = QRadioButton("1")
        self.goal_priority_2 = QRadioButton("2")
        self.goal_priority_3 = QRadioButton("3")
        self.start_task_button = QPushButton("작업 시작")

        task_layout.addWidget(QLabel("Red box:"), 0, 0)
        task_layout.addWidget(self.red_box_input, 0, 1)
        task_layout.addWidget(QLabel("개"), 0, 2)

        task_layout.addWidget(QLabel("Blue box:"), 1, 0)
        task_layout.addWidget(self.blue_box_input, 1, 1)
        task_layout.addWidget(QLabel("개"), 1, 2)

        goal_layout = QHBoxLayout()
        goal_layout.addWidget(QLabel("목표 위치: "))
        goal_layout.addWidget(self.goal_priority_1)
        goal_layout.addWidget(self.goal_priority_2)
        goal_layout.addWidget(self.goal_priority_3)

        task_layout.addLayout(goal_layout, 2, 0, 1, 4)  # 라디오 버튼 그룹을 한 줄에 정렬

        task_layout.addWidget(self.start_task_button, 3, 0, 1, 4)
        task_group.setLayout(task_layout)
        main_layout.addWidget(task_group)

        # Conveyor Section
        conveyor_group = QGroupBox("Conveyor 작동 상태")
        conveyor_layout = QHBoxLayout()
        self.conveyor_status_label=QLabel("컨베이어 상태: 작업 대기 중")

        self.ros_thread.ros_conveyor_status.connect(self.conveyor_status)
        conveyor_layout.addWidget(self.conveyor_status_label)
        conveyor_group.setLayout(conveyor_layout)
        main_layout.addWidget(conveyor_group)

        # Status Section
        status_layout = QGridLayout()
        self.image_check_button = QPushButton("확인")
        self.status_label = QLabel("현재 로봇 작업 상태:")
        self.status_display = QLabel("작업 대기중")
        self.time_label = QLabel("작업 시간:")
        self.time_display = QLCDNumber()
        self.time_display.display("00:00:00")

        status_layout.addWidget(QLabel("실시간 이미지:"), 0, 0)
        status_layout.addWidget(self.image_check_button, 0, 1)
        status_layout.addWidget(self.status_label, 1, 0)
        status_layout.addWidget(self.status_display, 1, 1)
        status_layout.addWidget(self.time_label, 2, 0)
        status_layout.addWidget(self.time_display, 2, 1)

        main_layout.addLayout(status_layout)
        
        # 좌표 표시 섹션 추가
        marker_pose_group = QGroupBox("현재 로봇 좌표")
        marker_pose_layout = QVBoxLayout()
        self.marker_pose_label = QLabel("X: 0.0, Y: 0.0, Z: 0.0")
        marker_pose_layout.addWidget(self.marker_pose_label)
        marker_pose_group.setLayout(marker_pose_layout)
        main_layout.addWidget(marker_pose_group)

        self.ros_thread.ros_image_signal.connect(self.display_frame)  # Connect ROS signal to frame display
        self.is_video_active = False  # 상태 변수 추가 (비디오 활성화 여부)
        # Add video feed section
        self.video_group = QGroupBox("실시간 비디오 피드")
        self.video_layout = QVBoxLayout()
        self.video_label = QLabel("비디오 피드 표시")
        self.video_label.setAlignment(Qt.AlignCenter)
        self.video_label.setStyleSheet("border: 2px solid #000; padding: 10px; background-color: #F0F0F0;")
        self.video_label.setFixedHeight(450)
        self.video_layout.addWidget(self.video_label)
        self.video_group.setLayout(self.video_layout)
        main_layout.addWidget(self.video_group)

        # Emergency Section
        emergency_layout = QHBoxLayout()
        
        self.stop_button = QPushButton("일시 정지")
        emergency_layout.addWidget(self.stop_button)
        self.resume_button = QPushButton("재시작")
        self.resume_button.setEnabled(False)
        emergency_layout.addWidget(self.resume_button)
        
        self.emergency_button = QPushButton("비상 정지")
        self.emergency_button.setStyleSheet("background-color: red;")
        emergency_layout.addWidget(self.emergency_button)
        self.reset_button = QPushButton("초기화")
        self.reset_button.setStyleSheet("background-color: red;")
        self.reset_button.setEnabled(False)
        emergency_layout.addWidget(self.reset_button)

        main_layout.addLayout(emergency_layout)

        # Set Main Layout
        self.setLayout(main_layout)

        # Timer for Task Time
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_time)
        self.total_elapsed_time = 0  # 멈춘 동안의 총 경과 시간을 저장
        self.start_time = None

        # Signal Connections
        self.start_task_button.clicked.connect(self.start_task)
        self.stop_button.clicked.connect(self.stop_task)
        self.emergency_button.clicked.connect(self.emergency_stop)
        self.resume_button.clicked.connect(self.resume_task)
        self.reset_button.clicked.connect(self.reset_task)
        self.image_check_button.clicked.connect(self.toggle_video_feed)
####################################################################################
        self.ros_node.marker_pose_signal.connect(self.update_marker_pose)

    def conveyor_control(self,state):
        self.ros_node.publish_step(state)

    def conveyor_status(self,status):
        if status=='PLAY':
            self.conveyor_status_label.setText("컨베이어 상태: 작동 중")
        elif status=="STOP":
            self.conveyor_status_label.setText("컨베이어 상태: 정지")
        else:
            self.conveyor_status_label.setText("컨베이어 상태: ERROR!!!")
            self.ros_node.get_logger().info("send-email to admin")
####################################################################################
    def toggle_video_feed(self):
        if not self.is_video_active:
            self.is_video_active = True
            self.image_check_button.setText("비디오 정지")
        else:
            self.is_video_active = False
            self.image_check_button.setText("확인")
            self.video_label.clear()
    
    def display_frame(self, frame):
        """Display frame in the QLabel."""
        if self.is_video_active:
            rgb_image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            h, w, ch = rgb_image.shape
            bytes_per_line = ch * w
            q_image = QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format_RGB888)
            pixmap = QPixmap.fromImage(q_image)
            self.video_label.setPixmap(pixmap.scaled(self.video_label.size(), Qt.KeepAspectRatio))
    def update_time(self):
        """Update the task time on the LCD."""
        if self.start_time:
            current_time = QDateTime.currentDateTime()
            elapsed = self.start_time.secsTo(current_time)  # 시작 시간부터 현재까지 경과 시간
            total_elapsed = elapsed + self.total_elapsed_time  # 누적 시간 포함
            hours = total_elapsed // 3600
            minutes = (total_elapsed % 3600) // 60
            seconds = total_elapsed % 60
            self.time_display.display(f"{hours:02}:{minutes:02}:{seconds:02}")

    def start_task(self):
        """Start a new task and reset the timer."""
        red_box = self.red_box_input.text()
        blue_box = self.blue_box_input.text()
        # Determine which goal priority is selected
        if self.goal_priority_1.isChecked():
            goal_priority = "1"
        elif self.goal_priority_2.isChecked():
            goal_priority = "2"
        elif self.goal_priority_3.isChecked():
            goal_priority = "3"
        else:
            goal_priority = None
        # Validate inputs
        if not red_box.isdigit() or not blue_box.isdigit():
            QMessageBox.warning(self, "Error", "Red box와 Blue box에 숫자를 입력해주세요.")
            return
        if goal_priority is None:
            QMessageBox.warning(self, "Error", "목표 위치를 선택해주세요.")
            return
        # Prepare task data
        box_list = f"Red: {red_box}, Blue: {blue_box}"
        start_time = QDateTime.currentDateTime().toString("yyyy-MM-dd HH:mm:ss")
        self.ros_node.publish_step(0)
        
        try:
            cursor.execute('''
                        INSERT INTO task (user_id, box_list, goal, start_time, status)
                        VALUES (?, ?, ?, ?, ?)
                        ''', (self.user[0], box_list, goal_priority, start_time, "in_progress"))
            conn.commit()
            # ROS 2 작업 데이터 발행
            self.ros_node.publish_box_data(red_box, blue_box)
            self.ros_node.publish_goal_priority(goal_priority)
            QMessageBox.information(self, "작업 실행", "작업이 성공적으로 시작되었습니다!")
        except sqlite3.Error as e:
            QMessageBox.critical(self, "에러", f"작업 실행 중 오류가 발생했습니다: {str(e)}")
        # 작업 타이머 시작
        self.start_time = QDateTime.currentDateTime()  # 작업 시작 시간 설정
        self.total_elapsed_time = 0  # 누적 시간 초기화
        self.timer.start(1000)  # 타이머 시작
        self.status_display.setText("작업중")

    def stop_task(self):
        """Pause the current task."""
        if not self.start_time:
            QMessageBox.warning(self, "Error", "현재 진행 중인 작업이 없습니다.")
            return
        # 현재까지의 경과 시간 계산
        current_time = QDateTime.currentDateTime()
        elapsed = self.start_time.secsTo(current_time)
        self.total_elapsed_time += elapsed  # 누적 시간에 추가
        self.start_time = None  # 시작 시간 초기화
        self.timer.stop()  # 타이머 중지
        self.status_display.setText("작업 정지")
        self.resume_button.setEnabled(True)
        self.ros_node.publish_step(1)
        
        try:
            # Update task status in the database
            cursor.execute('''
                UPDATE task
                SET status = ?
                WHERE user_id = ? AND status = 'in_progress'
            ''', ('stopped', self.user[0]))
            conn.commit()
            QMessageBox.information(self, "작업 일시 정지", "작업이 일시 정지되었습니다!")
        except sqlite3.Error as e:
            QMessageBox.critical(self, "에러", f"작업 일시 정지 중 오류가 발생했습니다: {str(e)}")

    def emergency_stop(self):
        if not self.start_time:
            QMessageBox.warning(self, "Error", "현재 진행 중인 작업이 없습니다.")
            return
        else:
            """Handle the emergency stop action."""
            self.timer.stop()
            self.status_display.setText("작업 비상정지")
            self.reset_button.setEnabled(True)
            self.ros_node.publish_step(1)
            try:
                # Update the database
                cursor.execute('''
                    UPDATE task
                    SET status = ?
                    WHERE user_id = ? AND status = 'in_progress'
                ''', (
                    'emergency_stop',                         # status
                    self.user[0]                              # user_id
                ))
                conn.commit()

                QMessageBox.information(self, "작업 비상정지", "작업이 비상정지 되었습니다.")
                ####################################################################################
                # 발신자 및 수신자 정보
                email = cursor.execute("SELECT admin_email FROM admin;").fetchone()[0]
                send_email_gmail(
                    sender_email="tjdwocl85@gmail.com",  # Gmail 주소
                    sender_password="uupugqxxaklcrmzz",  # 앱 비밀번호
                    recipient_email= email,
                    subject="시스템 비상 정지",
                    body="시스템이 비상 정지 되었습니다. 시스템을 확인해주십시오!"
                )
                ####################################################################################
            except sqlite3.Error as e:
                QMessageBox.critical(self, "에러", f"작업 비상정지 중 오류가 발생했습니다: {str(e)}")

    def resume_task(self):
        """Resume the task from where it stopped."""
        if self.start_time:
            QMessageBox.warning(self, "Error", "현재 정지된 작업이 없습니다.")
            return
        # 작업 재시작 시간 설정
        self.start_time = QDateTime.currentDateTime()
        self.timer.start(1000)  # 타이머 시작
        self.status_display.setText("작업중")
        self.resume_button.setEnabled(False)
        self.reset_button.setEnabled(False)
        self.ros_node.publish_step(0)
        try:
            # Update task status in the database
            cursor.execute('''
                UPDATE task
                SET status = ?
                WHERE user_id = ? AND status = 'stopped'
            ''', ('in_progress', self.user[0]))
            conn.commit()
            QMessageBox.information(self, "작업 재시작", "작업이 재시작되었습니다!")
        except sqlite3.Error as e:
            QMessageBox.critical(self, "에러", f"작업 재시작 중 오류가 발생했습니다: {str(e)}")
    def reset_task(self):
        """Reset the task and record its completion."""
        self.timer.stop()
        self.start_time = None
        self.time_display.display("00:00:00")
        self.status_display.setText("대기중")
        self.resume_button.setEnabled(False)
        self.reset_button.setEnabled(False)
        end_time = QDateTime.currentDateTime().toString("yyyy-MM-dd HH:mm:ss")
        try:
            # Update the database
            cursor.execute('''
                UPDATE task
                SET end_time = ?, status = ?
                WHERE user_id = ? AND status = 'emergency_stop'
            ''', (end_time,
                'canceled',                         # status
                self.user[0]                              # user_id
            ))
            conn.commit()
            QMessageBox.information(self, "작업 취소", "작업이 취소 되었습니다.")
        except sqlite3.Error as e:
            QMessageBox.critical(self, "에러", f"작업 취소 중 오류가 발생했습니다: {str(e)}")

    def open_admin_settings(self):
        """Open the Admin Settings dialog."""
        admin_dialog = AdminSettings(self.ros_node)
        admin_dialog.exec_()
    
    def update_marker_pose(self, pose):
        """GUI에서 로봇 좌표 업데이트."""
        if len(pose) == 6:
            self.marker_pose_label.setText(f"X: {pose[0]:.2f}, Y: {pose[1]:.2f}, Z: {pose[2]:.2f}\nRx: {pose[3]:.2f}, Ry: {pose[4]:.2f}, Rz: {pose[5]:.2f}")
        else:
            self.marker_pose_label.setText("좌표 데이터가 올바르지 않습니다.")

class AdminSettings(QDialog):
    def __init__(self,ros_node):
        super().__init__()
        self.setWindowTitle("관리자 설정")
        self.setGeometry(400, 200, 500, 400)
        self.email = ""
        self.is_collecting = False  # 수집 상태를 저장
        self.ros_node = ros_node  # Ros2Node 객체를 직접 전달받음
        
        layout = QVBoxLayout()

        # Email Update Section
        layout.addWidget(QLabel("관리자 이메일 변경:"))
        self.email_input = QLineEdit()
        self.email_input.setPlaceholderText("새로운 이메일 입력")
        self.email_update_button = QPushButton("이메일 변경")
        self.email_update_button.clicked.connect(self.update_email)
        layout.addWidget(self.email_input)
        layout.addWidget(self.email_update_button)

        # Task Table Section
        self.task_table_button = QPushButton("작업 목록 보기")
        self.task_table_button.clicked.connect(self.show_task_table)
        layout.addWidget(self.task_table_button)

        # Data Collection Section
        self.data_collection_button = QPushButton("데이터 수집")
        self.data_collection_button.clicked.connect(self.collect_data)
        layout.addWidget(self.data_collection_button)

        self.setLayout(layout)

    def update_email(self):
        """Update admin email in the database."""
        new_email = self.email_input.text()
        if not new_email:
            QMessageBox.warning(self, "경고", "이메일을 입력하세요!")
            return

        # 테이블에 데이터가 있는지 확인
        cursor.execute("SELECT admin_email FROM admin LIMIT 1")
        result = cursor.fetchone()

        if result:
            # 데이터가 있다면 수정
            cursor.execute("UPDATE admin SET admin_email = ? WHERE admin_id = 1", (new_email,))
            QMessageBox.information(self, "성공", "이메일이 성공적으로 업데이트되었습니다!")
        else:
            # 데이터가 없다면 추가
            cursor.execute("INSERT INTO admin (admin_email) VALUES (?)", (new_email,))
            QMessageBox.information(self, "성공", "이메일이 성공적으로 추가되었습니다!")

        conn.commit()

    def show_task_table(self):
        """Display task information from the database."""
        cursor.execute("""SELECT task_id, user_id, box_list, goal, start_time, end_time, status FROM task""")
        tasks = cursor.fetchall()

        if not tasks:
            QMessageBox.information(self, "정보", "작업 데이터가 없습니다.")
            return

        table_dialog = QDialog(self)
        table_dialog.setWindowTitle("작업 목록")
        table_dialog.setGeometry(400, 300, 1000, 600)

        # QTableWidget 생성
        table = QTableWidget(len(tasks), 7)
        table.setHorizontalHeaderLabels([
            "Task ID", "User ID", "Box List", "Goal", 
            "Start Time", "End Time", "Status"
        ])

        for row_idx, task in enumerate(tasks):
            for col_idx, value in enumerate(task):
                table.setItem(row_idx, col_idx, QTableWidgetItem(str(value) if value is not None else ""))
            
        # 레이아웃 설정
        layout = QVBoxLayout()
        layout.addWidget(table)
        table_dialog.setLayout(layout)
        
        # 창 띄우기
        table_dialog.exec_()

    def collect_data(self):
        """Toggle data collection and publish to ROS2."""
        self.is_collecting = not self.is_collecting  # 상태 토글
        self.ros_node.publish_capture(self.is_collecting)  # ROS2 메시지 발행

        # 버튼 상태에 따라 메시지 박스 표시
        if self.is_collecting:
            QMessageBox.information(self, "데이터 수집", "데이터 수집을 시작합니다!")
        else:
            QMessageBox.information(self, "데이터 수집", "데이터 수집을 중지합니다!")

def main():
    # 해결: QT 플랫폼 플러그인 문제 방지
    import os
    os.environ["QT_QPA_PLATFORM_PLUGIN_PATH"] = "/usr/lib/qt/plugins"

    # ROS 2 Initialization
    rclpy.init()

    app = QApplication(sys.argv)
    ros_thread = Ros2Thread()
    ros_thread.start()

    window = LoginWindow(ros_thread)
    window.show()

    exit_code = app.exec()

    ros_thread.stop()
    sys.exit(exit_code)

if __name__=="__main__":
    main()