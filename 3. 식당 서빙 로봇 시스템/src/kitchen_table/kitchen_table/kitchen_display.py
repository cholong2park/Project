import sys
import threading
import queue
import rclpy
import time
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from builtin_interfaces.msg import Duration
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QPushButton, QVBoxLayout, QHBoxLayout,
    QGridLayout, QLabel, QListWidget, QWidget, QTextEdit, QDialog, QMessageBox, 
    QLineEdit, QTableWidget, QTableWidgetItem
)
from PyQt5.QtCore import Qt, QTimer
from std_msgs.msg import String
from srv_interface.srv import Order
import sqlite3
from datetime import datetime

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point, Quaternion
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from nav2_msgs.srv import SetInitialPose

menu = {
        "치킨": {"price": 18000, "image": "./src/images/chicken.jpg"},
        "피자": {"price": 28000, "image": "./src/images/pizza.jpeg"},
        "햄버거": {"price": 9000, "image": "./src/images/burger.jpg"},
        "스테이크": {"price": 20000, "image": "./src/images/steak.jpg"},
        "파스타": {"price": 12000, "image": "./src/images/pasta.jpeg"},
        "라자냐": {"price": 11000, "image": "./src/images/lasagna.jpg"},
        "비프 웰링턴": {"price": 65000, "image": "./src/images/beef_wellington.jpg"},
        "제로 콜라": {"price": 2000, "image": "./src/images/cola.jpg"},
        "스프라이트": {"price": 2000, "image": "./src/images/sprite.png"},
        "환타": {"price": 2000, "image": "./src/images/fanta.png"},
        "하이볼": {"price": 5000, "image": "./src/images/highball.jpeg"},
        "생맥주 500cc": {"price": 5000, "image": "./src/images/beer.jpg"},
        "소주": {"price": 6000, "image": "./src/images/soju.png"},
        }

class KitchenNode(Node):
    """
    ROS2 Node: 주방 기능 처리
    """
    def __init__(self, request_queue):
        super().__init__('kitchen_node')  # 노드 이름 설정
        self.request_queue = request_queue  # 메시지를 전달받을 큐 생성
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,  # 신뢰성: 메시지 손실 없이 전달
            durability=DurabilityPolicy.TRANSIENT_LOCAL,  # 내구성: 연결 끊김 후에도 메시지 보존
            history=HistoryPolicy.KEEP_LAST,  # 히스토리: 최신 메시지 유지
            depth=30  # 히스토리에서 유지할 메시지 개수
            )
        
        self.manual_response = "Wait"  # GUI에서 응답을 보내도록 설정

        # 테이블 요청 구독: 'table_request' 토픽에서 메시지를 받아 처리
        self.table_request_sub = self.create_subscription(
            String, 'table_request', self.handle_table_request, qos_profile)

        # 주문 서비스 서버: 'kitchen_order' 서비스 요청을 받아 처리
        self.order_service = self.create_service(Order, 'kitchen_order', self.handle_order)

        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.set_initial_pose_service_client = self.create_client(
            SetInitialPose,
            '/set_initial_pose'
            )
        self.moving_path = []  # 경로를 저장하는 리스트
        self.busy = False  # 로봇 상태 플래그
        self.init_pose = [-2.0, -0.5, 0.0, 1.0]
        while not self.set_initial_pose_service_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service /set_initial_pose not available, waiting again...')
        self.set_initial_pose(*self.init_pose)
        self.get_logger().info("KitchenNode가 초기화되었습니다.")

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
        if future.result() is not None:
            self.get_logger().info("[INFO] Initial pose set successfully")
        else:
            self.get_logger().info("[WARN] Failed to set initial pose")
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
        self.busy = False  # 목표 완료, 상태 해제
        time.sleep(3)
        self.send_next_goal()  # 다음 목표 전송

    def send_next_goal(self):
        if self.busy:
            self.get_logger().info("Robot is busy. Waiting for current goal to complete.")
            return

        if self.moving_path:
            next_goal = self.moving_path.pop(0)
            x, y, theta = next_goal
            self.busy = True  # 로봇 상태 설정
            self.send_goal(x, y, theta)
        else:
            self.get_logger().info("All goals have been reached.")

    def handle_table_request(self, msg):
        """
        테이블 요청 메시지 처리
        """
        self.get_logger().info(f"테이블 요청 수신: {msg.data}")  # 요청 수신 로그 출력
        self.request_queue.put(f"{msg.data}")  # 요청 내용을 Queue에 추가

    def handle_order(self, request, response):
        """
        주문 서비스 요청 처리
        """
        self.get_logger().info(f"주문 수신: 테이블 {request.table_num}, 메뉴 {request.menu}, 수량 {request.quantity}")
        self.request_queue.put(f"Table{request.table_num}: {request.menu} x {request.quantity}")  # 주문 내용을 Queue에 추가

        # 응답 대기 (최대 60초 대기)
        for _ in range(60):
            if self.manual_response == "Accepted":
                self.get_logger().info('주문을 수락하였습니다!')
                response.accept = True
                self.manual_response = "Wait"
                return response
            elif self.manual_response == "Aborted":
                self.get_logger().info('주문을 거절하였습니다!')
                response.accept = False
                self.manual_response = "Wait"
                return response
            elif self.manual_response == "Wait":
                pass
            time.sleep(1)
        
        # 타임아웃 처리
        self.get_logger().warning("주문 응답 시간이 초과되었습니다!")
        response.accept = False
        return response

    def set_accepted_response(self):
        """
        GUI에서 수동 응답 설정
        """
        self.manual_response = "Accepted"

    def set_aborted_response(self):
        """
        GUI에서 수동 응답 설정
        """
        self.manual_response = "Aborted"

class StatsWindow(QDialog):
    """
    통계 데이터를 표시하는 창
    """
    def __init__(self, db_path, parent=None):
        super().__init__(parent)
        self.setWindowTitle("통계 열람")
        self.setFixedSize(1500, 1000)

        # SQLite 데이터베이스 연결
        self.db_connection = sqlite3.connect(db_path)
        self.db_cursor = self.db_connection.cursor()

        # 레이아웃 설정
        layout = QVBoxLayout(self)

        # 필터 옵션
        filter_layout = QHBoxLayout()

        # 날짜별 조회
        self.date_label = QLabel("날짜별 조회:")
        self.date_input = QLineEdit()
        self.date_input.setPlaceholderText("YYYY-MM-DD")
        self.date_search_button = QPushButton("검색")
        self.date_search_button.clicked.connect(self.filter_by_date)

        # 시간 구간 조회
        self.time_start_label = QLabel("시작 시간:")
        self.time_start_input = QLineEdit()
        self.time_start_input.setPlaceholderText("HH:MM:SS")

        self.time_end_label = QLabel("종료 시간:")
        self.time_end_input = QLineEdit()
        self.time_end_input.setPlaceholderText("HH:MM:SS")

        self.time_search_button = QPushButton("검색")
        self.time_search_button.clicked.connect(self.filter_by_time_range)

        # 메뉴별 조회
        self.menu_search_button = QPushButton("메뉴별 조회")
        self.menu_search_button.clicked.connect(self.filter_by_menu)

        # 월별 조회
        self.month_search_button = QPushButton("월별 조회")
        self.month_search_button.clicked.connect(self.filter_by_month)

        # 일별 조회
        self.day_search_button = QPushButton("일별 조회")
        self.day_search_button.clicked.connect(self.filter_by_hour)

        # 필터 옵션 배치
        filter_layout.addWidget(self.date_label)
        filter_layout.addWidget(self.date_input)
        filter_layout.addWidget(self.date_search_button)
        filter_layout.addWidget(self.time_start_label)
        filter_layout.addWidget(self.time_start_input)
        filter_layout.addWidget(self.time_end_label)
        filter_layout.addWidget(self.time_end_input)
        filter_layout.addWidget(self.time_search_button)
        filter_layout.addWidget(self.menu_search_button)
        filter_layout.addWidget(self.month_search_button)
        filter_layout.addWidget(self.day_search_button)

        layout.addLayout(filter_layout)

        # 데이터 테이블
        self.table_widget = QTableWidget()
        self.table_widget.setSortingEnabled(True)  # 정렬 활성화
        layout.addWidget(self.table_widget)

        # 데이터 로드
        self.load_all_data()

    def load_all_data(self):
        """
        데이터베이스에서 모든 데이터를 로드하여 테이블에 표시
        """
        query = "SELECT menu_name, quantity, order_date, order_time, total_price FROM orders"
        self.update_table_headers(["메뉴 이름", "수량", "날짜", "시간", "총 매출"])
        self.populate_table(query)

    def update_table_headers(self, headers):
        """
        테이블 헤더를 업데이트
        """
        self.table_widget.setColumnCount(len(headers))
        self.table_widget.setHorizontalHeaderLabels(headers)

    def filter_by_date(self):
        """
        입력된 날짜로 데이터를 필터링
        """
        date = self.date_input.text().strip()
        if date:
            query = "SELECT menu_name, quantity, order_date, order_time, total_price FROM orders WHERE order_date = ?"
            self.update_table_headers(["메뉴 이름", "수량", "날짜", "시간", "총 매출"])
            self.populate_table(query, (date,))
        else:
            QMessageBox.warning(self, "경고", "날짜를 입력하세요.")

    def filter_by_time_range(self):
        """
        입력된 시간 구간으로 데이터를 필터링
        """
        time_start = self.time_start_input.text().strip()
        time_end = self.time_end_input.text().strip()
        if time_start and time_end:
            query = """SELECT menu_name, quantity, order_date, order_time, total_price 
                       FROM orders WHERE order_time BETWEEN ? AND ?"""
            self.update_table_headers(["메뉴 이름", "수량", "날짜", "시간", "총 매출"])
            self.populate_table(query, (time_start, time_end))
        else:
            QMessageBox.warning(self, "경고", "시간 구간을 입력하세요.")

    def filter_by_menu(self):
        """
        메뉴별로 수량과 총 매출을 합산하여 표시
        """
        query = """
        SELECT strftime('%Y-%m', order_date) AS year_month, menu_name, 
               SUM(quantity) AS total_quantity, SUM(total_price) AS total_price
        FROM orders
        GROUP BY year_month, menu_name
        """
        self.update_table_headers(["년-월", "메뉴 이름", "수량", "총 매출"])
        self.populate_table(query)

    def filter_by_month(self):
        """
        월별 매출을 계산하여 표시
        """
        query = """
        SELECT strftime('%Y-%m', order_date) AS month, 
            SUM(quantity) AS total_quantity, 
            SUM(total_price) AS total_sales
        FROM orders
        GROUP BY month
        """
        self.update_table_headers(["월", "수량", "총 매출"])
        self.populate_table(query)

    def filter_by_hour(self):
        """
        선택한 날짜에 대해 1시간 간격으로 데이터를 조회
        """
        date = self.date_input.text().strip()
        if date:
            query = """
            SELECT strftime('%H:00', order_time) AS hour, SUM(quantity) AS total_quantity, 
                   SUM(total_price) AS total_price
            FROM orders
            WHERE order_date = ?
            GROUP BY hour
            """
            self.update_table_headers(["시간", "수량", "시간별 총 매출"])
            self.populate_table(query, (date,))
        else:
            QMessageBox.warning(self, "경고", "날짜를 입력하세요.")

    def populate_table(self, query, params=()):
        """
        주어진 쿼리의 결과로 테이블을 채움
        """
        self.db_cursor.execute(query, params)
        results = self.db_cursor.fetchall()

        # 테이블 설정
        self.table_widget.setRowCount(len(results))
        for row_idx, row_data in enumerate(results):
            for col_idx, value in enumerate(row_data):
                item = QTableWidgetItem(str(value) if value is not None else "")
                self.table_widget.setItem(row_idx, col_idx, item)

    def closeEvent(self, event):
        """
        창 닫힐 때 데이터베이스 연결 닫기
        """
        self.db_connection.close()
        event.accept()

class KitchenGUI(QMainWindow):
    """
    PyQt5 GUI: 주방 관리 UI
    """
    def __init__(self, ros_node, request_queue):
        super().__init__()
        self.ros_node = ros_node  # ROS2 노드 인스턴스
        self.request_queue = request_queue  # 요청 큐

        self.setWindowTitle("Kitchen Display")  # 윈도우 제목 설정
        self.setGeometry(100, 100, 2560, 1280)  # 윈도우 크기 설정

        # SQLite3 데이터베이스 경로 설정
        self.db_path = "./src/restaurant.db"
        # ros_node.get_logger().info(f"Using database path: {db_path}")  # ROS2 노드 로거로 경로 출력
        
        # 데이터베이스 연결
        self.db_connection = sqlite3.connect(self.db_path) # 데이터베이스 연결
        self.db_cursor = self.db_connection.cursor()

        # 로봇 경로와 테이블 데이터 초기화
        self.robot_path = []  # 로봇 경로

        self.robot_path_num = [] # 전달할 로봇 경로
        self.table_coordinates = {
            1: (-0.7, 1.8, 0.0),  # 테이블 1
            2: (0.8, 1.8, 0.0),  # 테이블 2
            3: (1.8, 0.6, 0.0),  # 테이블 3
            4: (1.8, -0.6, 0.0),  # 테이블 4
            5: (0.7, -1.8, 0.0),  # 테이블 5
            6: (-0.6, -1.8, 0.0),  # 테이블 6
        }

        self.tables_data = {f"Table{i}": {} for i in range(1, 7)}  # 테이블별 데이터 저장

        # 메인 레이아웃 설정
        main_layout = QHBoxLayout()

        # 좌측 레이아웃: 테이블 및 로봇 경로 관리
        left_layout = QVBoxLayout()

        # 여백 및 간격 설정
        left_layout.setContentsMargins(0, 0, 0, 0)  # 레이아웃 외부 여백 제거
        left_layout.setSpacing(5)  # 위젯 사이 간격 제거

        # 로봇 경로 텍스트박스
        self.robot_path_display = QTextEdit()
        self.robot_path_display.setReadOnly(True)  # 읽기 전용
        self.robot_path_display.setFixedSize(600, 100)  # 너비 600, 높이 100으로 고정
        left_layout.addWidget(QLabel("로봇 경로"))
        left_layout.addWidget(self.robot_path_display, stretch=1)

        # 테이블 버튼 (6개)
        table_layout = QGridLayout()
        self.table_buttons = {}
        self.clear_buttons = {}  # 결제 버튼 저장

        for i in range(6):
            table_button = QPushButton(f"Table{i+1}\n\n")  # 테이블 정보 버튼
            table_button.setFixedSize(300, 200)  # 버튼 크기 설정
            table_button.clicked.connect(lambda _, t=f"Table{i+1}", num=i+1: self.toggle_table(t, num))  # 클릭 이벤트
            self.table_buttons[f"Table{i+1}"] = table_button  # 버튼 저장
            table_layout.addWidget(table_button, i // 2, i % 2)  # 2열 그리드 배치
        
             # 결제 버튼
            clear_button = QPushButton(f"결제 (Table{i+1})")  # 결제 버튼 생성
            clear_button.setFixedSize(300, 50)  # 결제 버튼 크기 설정
            clear_button.clicked.connect(lambda _, t=f"Table{i+1}": self.clear_table_data(t))  # 클릭 이벤트
            self.clear_buttons[f"Table{i+1}"] = clear_button  # 결제 버튼 저장
            table_layout.addWidget(clear_button, i // 2 + 3, i % 2)  # 결제 버튼 추가
        left_layout.addLayout(table_layout, stretch=9)

        # 좌측 레이아웃을 메인 레이아웃에 추가 (50% 비율)
        main_layout.addLayout(left_layout, 5)

        # 중앙 레이아웃: 요청 및 메뉴 접수
        center_layout = QVBoxLayout()

        # 요청 리스트
        self.request_list = QListWidget()
        self.request_list.itemClicked.connect(self.handle_request_completion)  # 요청 완료 처리
        center_layout.addWidget(QLabel("요청 사항"))
        center_layout.addWidget(self.request_list, stretch=1)

        # 메뉴 접수 리스트
        self.menu_list = QListWidget()
        self.menu_list.itemClicked.connect(self.handle_menu_selection)  # 메뉴 선택 처리
        center_layout.addWidget(QLabel("메뉴 접수"))
        center_layout.addWidget(self.menu_list, stretch=2)

        # 중앙 레이아웃을 메인 레이아웃에 추가 (40% 비율)
        main_layout.addLayout(center_layout, 4)

        # 우측 레이아웃: 통계 및 로봇 제어
        right_layout = QVBoxLayout()
        right_layout.setAlignment(Qt.AlignCenter)
        # 버튼 간 간격 추가
        button_spacing = 20

        # 통계 버튼
        stats_button = QPushButton("통계")
        stats_button.setFixedSize(120, 40)  # 크기 설정
        stats_button.clicked.connect(self.open_stats_window)  # 통계 창 연결
        right_layout.addWidget(stats_button)
        right_layout.addSpacing(button_spacing)

        # 편집 버튼
        edit_button = QPushButton("편집")
        edit_button.setFixedSize(120, 40)
        edit_button.clicked.connect(self.edit_table_menu)
        right_layout.addWidget(edit_button)
        right_layout.addSpacing(button_spacing)

        # 호출 버튼
        self.robot_call_button = QPushButton("로봇 호출")
        self.robot_call_button.clicked.connect(self.call_robot)
        self.robot_call_button.setFixedSize(120, 40)
        right_layout.addWidget(self.robot_call_button)
        right_layout.addSpacing(button_spacing)

        # 로봇 출발 버튼
        self.robot_start_button = QPushButton("로봇 출발")
        self.robot_start_button.clicked.connect(self.send_path_robot)
        self.robot_start_button.setFixedSize(120, 40)
        right_layout.addWidget(self.robot_start_button)

        # 우측 레이아웃을 메인 레이아웃에 추가 (10% 비율)
        main_layout.addLayout(right_layout, 1)

        # 메인 레이아웃을 중앙 위젯에 적용
        central_widget = QWidget()
        central_widget.setLayout(main_layout)
        self.setCentralWidget(central_widget)

        # QTimer를 사용해 요청 확인
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.check_requests)  # 요청 확인
        self.timer.start(100)  # 100ms마다 실행

    def call_robot(self):
        self.ros_node.moving_path.append((-2.0, -0.5, 0.0))  # KitchenNode의 moving_path에 전달
        self.ros_node.send_next_goal()

    def send_path_robot(self):
        if not self.robot_path_num:
            self.ros_node.get_logger().warn("No waypoints to send.")
            return

        # 로봇 경로를 moving_path로 전달
        for num in self.robot_path_num:
            x,y,theta = self.table_coordinates[num]
            self.ros_node.moving_path.append((x, y, theta))  # KitchenNode의 moving_path에 전달
        self.ros_node.moving_path.append((-2.0, 0.5, 0.0))  
        self.robot_path_num = []  # 초기화
        self.robot_path = []
        self.update_robot_path_display()

        # 첫 번째 목표 전송
        self.ros_node.send_next_goal()
        
    def open_stats_window(self):
        """
        통계 창 열기
        """
        stats_window = StatsWindow(self.db_path, self)
        stats_window.exec_()

    def save_order_to_db(self, menu_name, quantity):
        """
        SQLite3 데이터베이스에 주문 저장
        """
        global menu

        now = datetime.now()  # 현재 날짜와 시간 가져오기
        order_date = now.strftime("%Y-%m-%d")  # 날짜 포맷
        order_time = now.strftime("%H:%M:%S")  # 시간 포맷
        total_price = quantity * menu[menu_name]["price"]

        # 데이터베이스에 삽입
        self.db_cursor.execute("""
            INSERT INTO orders (menu_name, quantity, order_date, order_time, total_price)
            VALUES (?, ?, ?, ?, ?);
        """, (menu_name, quantity, order_date, order_time, total_price))
        self.db_connection.commit()  # 변경 사항 저장

    def toggle_table(self, table_name, table_num):
        """테이블 클릭 시 로봇 경로 추가/삭제"""
        if table_name in self.robot_path:
            self.robot_path.remove(table_name)  # 경로에서 제거
            self.robot_path_num.remove(table_num)
        else:
            self.robot_path.append(table_name)  # 경로에 추가
            self.robot_path_num.append(table_num)
        self.update_robot_path_display()


    def update_robot_path_display(self):
        """로봇 경로 실시간 업데이트"""
        self.robot_path_display.setText(" -> ".join(self.robot_path))  # 경로 표시

    def handle_request_completion(self, item):
        """요청 사항 완료 처리"""
        if item:
            reply = QMessageBox.question(
                self, "요청 완료", f"{item.text()}를 완료하시겠습니까?",
                QMessageBox.Yes | QMessageBox.No
            )
            if reply == QMessageBox.Yes:
                self.request_list.takeItem(self.request_list.row(item))  # 요청 리스트에서 제거

    def handle_menu_selection(self, item):
        """메뉴 접수/거절 처리"""
        if item:
            dialog = QDialog(self)
            dialog.setWindowTitle("메뉴 처리")
            layout = QVBoxLayout(dialog)

            label = QLabel(f"{item.text()}를 어떻게 처리하시겠습니까?")
            layout.addWidget(label)

            accept_button = QPushButton("접수")
            accept_button.clicked.connect(lambda: self.process_menu(item, True, dialog))
            
            reject_button = QPushButton("거절")
            reject_button.clicked.connect(lambda: self.process_menu(item, False, dialog))

            layout.addWidget(accept_button)
            layout.addWidget(reject_button)
            dialog.setLayout(layout)
            dialog.exec_()

    def process_menu(self, item, action, dialog):
        """
        메뉴를 접수 또는 거절
        """
        if action:
            table_name, menu_data = item.text().split(": ", 1)
            menu_name, quantity_str = menu_data.split(" x ")
            quantity = int(quantity_str)  # 수량 변환
            
            # 기존 메뉴가 있는지 확인하고 수량 합산
            if menu_name in self.tables_data[table_name]:
                self.tables_data[table_name][menu_name] += quantity
            else:
                self.tables_data[table_name][menu_name] = quantity
            
            # 테이블 UI 업데이트
            self.update_table_display(table_name)

            # SQLite3에 주문 저장
            self.save_order_to_db(menu_name, quantity)
            
            # ROS2 노드에 접수 완료 상태 알림
            self.ros_node.get_logger().info(f"{table_name}의 {menu_name} x{quantity} 접수 완료.")
            self.ros_node.set_accepted_response()  # ROS2 노드의 수동 응답 플래그 설정
        
        else:  # 거절 처리
            self.ros_node.get_logger().info(f"{item.text()}가 거절되었습니다.")
            self.ros_node.set_aborted_response()  # ROS2 노드의 수동 응답 플래그 설정
        
        self.menu_list.takeItem(self.menu_list.row(item))
        dialog.accept()

    def closeEvent(self, event):
        """
        GUI 종료 시 데이터베이스 연결 닫기
        """
        self.db_connection.close()
        event.accept()

    def update_table_display(self, table_name):
        """
        테이블 메뉴 정보 업데이트
        """
        menu_info = "\n".join(
            f"{menu_name} - 수량: {quantity}"
            for menu_name, quantity in self.tables_data[table_name].items()
        )
        self.table_buttons[table_name].setText(f"{table_name}\n\n{menu_info}")

    def check_requests(self):
        """Queue에서 요청 확인"""
        while not self.request_queue.empty():
            request = self.request_queue.get()
            if "Table 요청" in request:
                self.request_list.addItem(request)  # 요청 리스트에 추가
            else:
                self.menu_list.addItem(request)  # 메뉴 리스트에 추가

    def clear_table_data(self, table_name):
        """
        특정 테이블 데이터를 초기화하고 결제 금액을 보여주는 팝업창을 표시
        """
        # 테이블에 있는 메뉴들의 총 금액 계산
        total_amount = 0
        for menu_name, quantity in self.tables_data[table_name].items():
            total_amount += menu[menu_name]["price"] * quantity  # 가격 * 수량

        # 결제 금액 팝업창 표시
        reply = QMessageBox.question(
            self, "결제", f"{table_name}의 결제 금액은 {total_amount}원입니다. 결제를 완료하시겠습니까?",
            QMessageBox.Yes | QMessageBox.No
        )

        # 결제 완료 시 데이터 초기화
        if reply == QMessageBox.Yes:
            self.tables_data[table_name] = {}  # 데이터 초기화
            self.update_table_display(table_name)  # UI 업데이트
            QMessageBox.information(self, "완료", f"{table_name}의 결제가 완료되었습니다.")

    def edit_table_menu(self):
        """
        테이블 메뉴 편집: 이동할 소스 테이블과 대상 테이블을 선택
        """
        dialog = QDialog(self)
        dialog.setWindowTitle("테이블 메뉴 편집")
        dialog_layout = QVBoxLayout(dialog)

        label = QLabel("먼저 이동할 메뉴가 있는 테이블을 선택하세요.")
        dialog_layout.addWidget(label)

        # 소스 테이블 버튼 생성
        for table_name in self.tables_data.keys():
            button = QPushButton(f"{table_name}")
            button.clicked.connect(lambda _, t=table_name, d=dialog: self.select_source_table(t, d))
            dialog_layout.addWidget(button)

        dialog.setLayout(dialog_layout)
        dialog.exec_()

    def select_source_table(self, source_table, dialog):
        """
        소스 테이블 선택 후 대상 테이블 선택 다이얼로그 열기
        """
        if not self.tables_data[source_table]:  # 소스 테이블에 메뉴가 없을 경우
            QMessageBox.warning(self, "오류", f"{source_table}에는 메뉴가 없습니다.")
            return

        # 대상 테이블 선택 다이얼로그 생성
        target_dialog = QDialog(self)
        target_dialog.setWindowTitle("대상 테이블 선택")
        target_dialog_layout = QVBoxLayout(target_dialog)

        label = QLabel(f"{source_table}에서 메뉴를 이동할 대상 테이블을 선택하세요.")
        target_dialog_layout.addWidget(label)

        # 대상 테이블 버튼 생성
        for table_name in self.tables_data.keys():
            if table_name != source_table:  # 소스 테이블과 다른 테이블만 표시
                button = QPushButton(f"{table_name}")
                button.clicked.connect(lambda _, s=source_table, t=table_name, d=target_dialog: self.move_menu_to_table(s, t, d))
                target_dialog_layout.addWidget(button)

        target_dialog.setLayout(target_dialog_layout)
        target_dialog.exec_()
    
    def move_menu_to_table(self, source_table, target_table, dialog):
        """
        소스 테이블에서 대상 테이블로 메뉴 이동
        """
        # 메뉴 데이터를 합산하여 이동
        for menu_name, quantity in self.tables_data[source_table].items():
            if menu_name in self.tables_data[target_table]:
                self.tables_data[target_table][menu_name] += quantity
            else:
                self.tables_data[target_table][menu_name] = quantity

        # 소스 테이블 데이터 초기화
        self.tables_data[source_table] = {}

        # UI 업데이트
        self.update_table_display(source_table)
        self.update_table_display(target_table)

        QMessageBox.information(self, "완료", f"{source_table}의 메뉴가 {target_table}로 이동되었습니다.")
        dialog.accept()


def main():
    """
    메인 실행 함수
    """
    rclpy.init()  # ROS2 초기화
    request_queue = queue.Queue()  # 요청 큐 생성
    ros_node = KitchenNode(request_queue)  # ROS 노드 생성

    # ROS2 스레드 관리
    running = threading.Event()
    running.set()

    def ros_spin():
        while running.is_set():
            rclpy.spin_once(ros_node, timeout_sec=0.1)  # ROS 메시지 처리

    ros_thread = threading.Thread(target=ros_spin, daemon=True)  # ROS 스레드 생성
    ros_thread.start()

    # PyQt5 실행
    app = QApplication(sys.argv)
    gui = KitchenGUI(ros_node, request_queue)
    gui.show()

    try:
        sys.exit(app.exec_())  # PyQt 애플리케이션 실행
    except KeyboardInterrupt:
        print("종료 중...")
    finally:
        running.clear()  # 스레드 종료
        ros_thread.join()
        ros_node.destroy_node()  # 노드 종료
        rclpy.shutdown()  # ROS2 종료

if __name__ == "__main__":
    main()
