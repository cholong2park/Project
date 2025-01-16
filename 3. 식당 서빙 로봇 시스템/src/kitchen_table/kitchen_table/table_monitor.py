import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from srv_interface.srv import Order
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from builtin_interfaces.msg import Duration

import sys
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QListWidget, QStackedWidget, QPushButton,
    QVBoxLayout, QLabel, QDialog, QHBoxLayout, QWidget, QGridLayout, QScrollArea,
    QListWidgetItem, QMessageBox
)
from PyQt5.QtGui import QFont, QPixmap
from PyQt5.QtCore import Qt, QTimer

class TableMonitorNode(Node):
    """
    ROS2 노드: 테이블 모니터 역할을 수행합니다.
    - 고객의 요청 사항을 퍼블리시합니다.
    - 주방에 주문을 서비스 요청합니다.
    - 배달 확인 요청에 대한 서비스를 제공합니다.
    """
    def __init__(self):
        super().__init__('table_monitor')  # 노드 이름 설정
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,  # 신뢰성: 메시지 손실 없이 전달
            durability=DurabilityPolicy.TRANSIENT_LOCAL,  # 내구성: 연결 끊김 후에도 메시지 보존
            history=HistoryPolicy.KEEP_LAST,  # 히스토리: 최신 메시지 유지
            depth=30  # 히스토리에서 유지할 메시지 개수
            )

        # Publisher: 고객 요청사항을 'table_request' 토픽에 발행
        self.publisher = self.create_publisher(String, 'table_request',qos_profile)

        # Client: 'kitchen_order' 서비스에 주문 요청
        self.request_client = self.create_client(Order, 'kitchen_order')
        self.answer = "Wait"

        self.get_logger().info("Table Monitor가 실행 중입니다")

    def publish_request(self, request_type):
        """
        고객 요청사항을 'table_request' 토픽에 발행합니다.
        :param request_type: 고객이 요청한 서비스 (예: '물', '앞치마' 등)
        """
        msg = String()
        msg.data = request_type  # 요청 내용을 메시지에 담기
        self.publisher.publish(msg)  # 메시지 발행
        self.get_logger().info(f"고객 요청사항 발행: {request_type}")

    def send_order_request(self, table_num, menu, quantity):
        """
        주방에 주문 요청을 전송합니다.
        :param table_num: 테이블 번호
        :param menu: 주문한 메뉴 이름
        :param quantity: 주문 수량
        """
        # 주문 서비스가 사용 가능한지 확인
        if not self.request_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error("주문 서비스가 현재 사용 불가능합니다!")
            return

        # 주문 요청 생성
        request = Order.Request()
        request.table_num = table_num
        request.menu = menu
        request.quantity = quantity

        # 주문 서비스 호출
        future = self.request_client.call_async(request)
        future.add_done_callback(self.confirm_response_callback)  # 완료 시 콜백 호출

    def confirm_response_callback(self, future):
        # 응답 처리
        if future.result():
            response = future.result()
            if response.accept:
                self.get_logger().info(f"주문 접수 완료")
                self.answer = "Accepted"
            else:
                self.get_logger().warning("주문이 거절되었습니다!")
                self.answer = "Aborted"
        else:
            self.get_logger().error("서비스 응답 실패!")

    def handle_response(self, request, response):
        """
        배달 확인 요청에 응답합니다.
        :param request: 배달 확인 요청 (메뉴 정보 포함)
        :param response: 배달 확인 응답 (수락 여부)
        """
        self.get_logger().info(f"배달 확인 요청: {request.menu}")
        response.accept = True  # 배달을 수락으로 설정
        self.get_logger().info("배달 확인 응답 전송 완료")
        return response  # 응답 반환


class MainWindow(QMainWindow):
    """
    PyQt5 기반의 GUI 클래스: 테이블 모니터 역할을 수행합니다.
    - 메뉴 선택 및 주문 기능을 제공합니다.
    - 요청 사항을 전송할 수 있습니다.
    - 장바구니를 관리하고 결제 기능을 제공합니다.
    """
    def __init__(self, ros_node):
        super().__init__()
        self.cart = []  # 장바구니 데이터 저장
        self.table_num = 1  # 테이블 번호 (고정값으로 설정되어 있으나 필요에 따라 변경 가능)
        self.ros_node = ros_node  # ROS2 노드 연결
        
        # QTimer 설정: 100ms마다 answer 상태 확인
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.check_ros_answer)
        self.timer.start(100)  # 100ms 주기 실행
        
        # QTimer로 ROS2 이벤트 루프 관리
        self.timer2 = QTimer(self)
        self.timer2.timeout.connect(self.spin_ros)  # QTimer가 ROS2 이벤트 루프를 실행
        self.timer2.start(10)  # 10ms 간격으로 실행

        # GUI 설정
        self.setWindowTitle("Restaurant Serving Robot")  # 윈도우 제목 설정
        self.setGeometry(100, 100, 2560, 1280)  # 윈도우 크기 및 위치 설정

        # 중앙 위젯과 레이아웃 설정
        self.central_widget = QWidget(self)
        self.setCentralWidget(self.central_widget)
        main_layout = QHBoxLayout(self.central_widget)  # 전체 수평 레이아웃 설정

        # 왼쪽 영역: 메뉴 카테고리 및 요청사항 버튼
        left_layout = QVBoxLayout()
        self.menu_list = QListWidget()  # 메뉴 카테고리 리스트 위젯
        self.menu_list.addItems(["메인", "음료", "주류"])  # 카테고리 추가
        self.menu_list.itemClicked.connect(self.change_menu)  # 카테고리 선택 시 이벤트 연결
        left_layout.addWidget(self.menu_list)

        self.request_button = QPushButton("요청사항")  # 요청사항 버튼
        self.request_button.clicked.connect(self.open_request_dialog)  # 클릭 시 이벤트 연결
        left_layout.addWidget(self.request_button)

        left_container = QWidget()
        left_container.setLayout(left_layout)
        main_layout.addWidget(left_container, stretch=1)  # 왼쪽 영역을 메인 레이아웃에 추가

        # 중앙 영역: 메뉴 선택 영역
        self.menu_stacked_widget = QStackedWidget()  # 스택 위젯을 사용하여 메뉴 페이지 전환
        self.setup_menu_pages()  # 메뉴 페이지 설정
        main_layout.addWidget(self.menu_stacked_widget, stretch=6)  # 중앙 영역을 메인 레이아웃에 추가

        # 오른쪽 영역: 장바구니 및 결제 버튼
        right_layout = QVBoxLayout()
        self.cart_widget = QListWidget()  # 장바구니 리스트 위젯
        self.cart_widget.itemClicked.connect(self.confirm_cancel)  # 장바구니 항목 클릭 시 취소 확인
        right_layout.addWidget(self.cart_widget)

        self.total_price_label = QLabel("총 가격: 0원")  # 총 가격 레이블 추가
        self.total_price_label.setFont(QFont("Arial", 12))  # 폰트 설정
        right_layout.addWidget(self.total_price_label)

        self.checkout_button = QPushButton("접수")  # 접수 버튼
        self.checkout_button.clicked.connect(self.checkout)  # 클릭 시 이벤트 연결
        right_layout.addWidget(self.checkout_button)

        right_container = QWidget()
        right_container.setLayout(right_layout)
        main_layout.addWidget(right_container, stretch=2)  # 오른쪽 영역을 메인 레이아웃에 추가

    def spin_ros(self):
        """ROS2 이벤트 루프 실행"""
        rclpy.spin_once(self.ros_node, timeout_sec=0.01)

    def check_ros_answer(self):
        """
        ROS2 노드의 answer 상태를 확인하고 팝업 표시
        """
        if self.ros_node.answer == "Accepted":
            QMessageBox.information(self, "주문 접수", "주문이 성공적으로 접수되었습니다!")
            self.ros_node.answer = "Wait"  # 상태 초기화
        elif self.ros_node.answer == "Aborted":
            QMessageBox.warning(self, "주문 거절", "주문이 거절되었습니다.")
            self.ros_node.answer = "Wait"  # 상태 초기화

    def setup_menu_pages(self):
        """
        각 메뉴 카테고리에 대한 페이지를 설정합니다.
        """
        # 메뉴 데이터 정의 (이미지 파일 경로 포함)
        menu = {
            "메인": [
                {"name": "치킨","price": 18000, "image": "./src/images/chicken.jpg"},
                {"name": "피자", "price": 28000, "image": "./src/images/pizza.jpeg"},
                {"name": "햄버거", "price": 9000, "image": "./src/images/burger.jpg"},
                {"name": "스테이크", "price": 20000, "image": "./src/images/steak.jpg"},
                {"name": "파스타", "price": 12000, "image": "./src/images/pasta.jpeg"},
                {"name": "라자냐", "price": 11000, "image": "./src/images/lasagna.jpg"},
                {"name": "비프 웰링턴","price": 65000, "image": "./src/images/beef_wellington.jpg"},
            ],
            "음료": [
                {"name": "제로 콜라", "price": 2000, "image": "./src/images/cola.jpg"},
                {"name": "스프라이트", "price": 2000, "image": "./src/images/sprite.png"},
                {"name": "환타", "price": 2000, "image": "./src/images/fanta.png"},
            ],
            "주류": [
                {"name": "하이볼", "price": 5000, "image": "./src/images/highball.jpeg"},
                {"name": "생맥주 500cc", "price": 5000, "image": "./src/images/beer.jpg"},
                {"name": "소주", "price": 6000, "image": "./src/images/soju.png"},
            ],
        }

        self.pages = {}  # 카테고리별 페이지를 저장할 딕셔너리

        for category, items in menu.items():
            # 스크롤 영역 생성
            scroll_area = QScrollArea()
            scroll_area.setWidgetResizable(True)  # 내용 크기에 따라 스크롤 조정
            
            page = QWidget()
            page_layout = QGridLayout(page)  # 2열 그리드 레이아웃
            page_layout.setAlignment(Qt.AlignTop)  # 위쪽 정렬

            for i, item in enumerate(items):
                # 이미지 생성
                image_label = QLabel()
                pixmap = QPixmap(item["image"])  # 이미지 로드
                pixmap = pixmap.scaled(500, 400, Qt.IgnoreAspectRatio)  # 크기 조정
                image_label.setPixmap(pixmap)

                # 버튼 생성
                menu_button = QPushButton(f'{item["name"]}\n가격: {item["price"]}')
                menu_button.setFixedSize(500, 100)  # 버튼 크기 설정
                menu_button.clicked.connect(lambda _, name=item["name"], price=item["price"]: self.open_quantity_dialog(name, price))

                # 이미지와 버튼을 수직으로 정렬
                container = QVBoxLayout()
                container.addWidget(image_label)
                container.addWidget(menu_button)

                # 컨테이너를 위젯으로 감싸고 그리드 레이아웃에 추가
                widget = QWidget()
                widget.setLayout(container)
                page_layout.addWidget(widget, i // 2, i % 2, alignment=Qt.AlignCenter)  # 2열로 배치

            # 스크롤 영역에 페이지 설정
            page.setLayout(page_layout)
            scroll_area.setWidget(page)

            # 페이지를 스택 위젯에 추가
            self.menu_stacked_widget.addWidget(scroll_area)
            self.pages[category] = scroll_area

    def change_menu(self, item):
        """
        메뉴 카테고리를 변경합니다.
        :param item: 선택된 QListWidgetItem (카테고리 이름)
        """
        category = item.text()
        if category in self.pages:
            self.menu_stacked_widget.setCurrentWidget(self.pages[category])  # 선택된 카테고리의 페이지로 전환

    def open_quantity_dialog(self, menu_name, price):
        """
        메뉴 수량을 결정하는 다이얼로그를 엽니다.
        :param menu_name: 선택된 메뉴의 이름
        """
        dialog = QDialog(self)
        dialog.setWindowTitle(f"{menu_name} 수량 결정")
        
        # 팝업창 크기 설정
        dialog.setFixedSize(400, 300)  # 팝업창 크기를 가로 400px, 세로 300px로 설정
        layout = QVBoxLayout(dialog)

        quantity_label = QLabel("수량: 1")  # 초기 수량은 1로 설정
        layout.addWidget(quantity_label)
        count = 1  # 수량을 저장할 변수

        # 수량 증가 함수
        def increase_quantity():
            nonlocal count
            count += 1
            quantity_label.setText(f"수량: {count}")

        # 수량 감소 함수
        def decrease_quantity():
            nonlocal count
            if count > 1:
                count -= 1
                quantity_label.setText(f"수량: {count}")

        # 수량 조절 버튼 생성
        plus_button = QPushButton("+")
        plus_button.clicked.connect(increase_quantity)
        minus_button = QPushButton("-")
        minus_button.clicked.connect(decrease_quantity)
        layout.addWidget(plus_button)
        layout.addWidget(minus_button)

        # 확인 버튼: 장바구니에 메뉴 추가
        confirm_button = QPushButton("확인")
        confirm_button.clicked.connect(lambda: [self.add_to_cart(menu_name, count, count * price), dialog.accept()])
        layout.addWidget(confirm_button)

        dialog.setLayout(layout)
        dialog.exec_()  # 다이얼로그 실행

    def add_to_cart(self, menu_name, count, money):
        """
        선택된 메뉴와 수량을 장바구니에 추가합니다.
        :param menu_name: 메뉴 이름
        :param count: 선택된 수량
        """
        for item in self.cart:
            if item['menu'] == menu_name:
                item['quantity'] += count  # 수량 업데이트
                item['all_price'] += money
                break
        else:
            self.cart.append({"menu": menu_name, "quantity": count, "all_price": money})
        # 장바구니 UI 업데이트
        self.update_cart_view()

    def confirm_cancel(self, item):
        """
        장바구니 항목 클릭 시 취소 여부를 묻는 다이얼로그를 띄웁니다.
        :param item: 클릭된 장바구니 항목
        """
        # 취소 여부를 묻는 메시지 박스 생성
        reply = QMessageBox.question(self, '취소 확인', f"{item.text()} 메뉴를 장바구니에서 삭제하시겠습니까?",
                                     QMessageBox.Yes | QMessageBox.No, QMessageBox.No)

        if reply == QMessageBox.Yes:
            self.remove_from_cart(item)  # 장바구니에서 항목 제거

    def remove_from_cart(self, item):
        """
        장바구니에서 메뉴 항목을 제거합니다.
        :param item: 제거할 장바구니 항목
        """
        cart_item = item.data(Qt.UserRole)  # 장바구니 아이템 데이터 가져오기
        self.cart.remove(cart_item)  # 장바구니에서 항목 제거
        self.update_cart_view()  # 장바구니 UI 업데이트

    def update_cart_view(self):
        """
        장바구니 리스트 위젯을 업데이트합니다.
        """
        self.cart_widget.clear()  # 기존 항목 제거
        total_price = 0  # 총 가격 변수 초기화
        for item in self.cart:
            cart_item_widget = QListWidgetItem(f"{item['menu']} - 수량: {item['quantity']} - 총 가격: {item['all_price']}")  # 장바구니 아이템 표시
            cart_item_widget.setData(Qt.UserRole, item)  # 아이템 데이터를 저장 (수정할 때 사용)
            self.cart_widget.addItem(cart_item_widget)
            total_price += item['all_price']  # 총 가격 업데이트

        self.total_price_label.setText(f"총 가격: {total_price}원")  # 총 가격 레이블 업데이트

    def checkout(self):
        """
        결제 버튼 클릭 시 호출됩니다.
        장바구니에 있는 모든 메뉴를 주문 요청으로 전송합니다.
        """
        for item in self.cart:
            self.ros_node.send_order_request(self.table_num, item['menu'], item['quantity'])  # 주문 요청 전송
        self.cart.clear()  # 장바구니 비우기
        self.update_cart_view()  # 장바구니 뷰 업데이트

    def open_request_dialog(self):
        """
        고객 요청사항을 선택할 수 있는 다이얼로그를 엽니다.
        """
        dialog = QDialog(self)
        dialog.setWindowTitle("요청사항")
        layout = QVBoxLayout(dialog)

        # 요청사항 목록
        requests = ["물", "앞치마", "앞접시", "물티슈", "숟가락 & 젓가락", "직원 호출"]
        active_requests = set()  # 선택된 요청사항을 저장할 집합

        # 요청사항 버튼 생성 및 클릭 이벤트 연결
        def toggle_request(button, request):
            if request in active_requests:
                active_requests.remove(request)
                button.setStyleSheet("background-color: none;")  # 선택 해제 시 배경색 제거
            else:
                active_requests.add(request)
                button.setStyleSheet("background-color: lightblue;")  # 선택 시 배경색 변경

        for request in requests:
            button = QPushButton(request)
            button.setCheckable(True)
            button.clicked.connect(lambda _, b=button, r=request: toggle_request(b, r))
            layout.addWidget(button)

        # 확인 버튼: 선택된 요청사항을 퍼블리시
        confirm_button = QPushButton("확인")
        confirm_button.clicked.connect(lambda: self.confirm_requests(dialog, active_requests))
        layout.addWidget(confirm_button)

        dialog.setLayout(layout)
        dialog.exec_()  # 다이얼로그 실행

    def confirm_requests(self, dialog, active_requests):
        """
        선택된 요청사항을 퍼블리시합니다.
        :param dialog: 현재 다이얼로그 객체
        :param active_requests: 선택된 요청사항 집합
        """
        for request in active_requests:
            message = f"{self.table_num}번 Table 요청: {request}"
            self.ros_node.publish_request(message)  # 요청사항 퍼블리시
        dialog.accept()  # 다이얼로그 닫기


def main(args=None):
    """
    프로그램의 진입점 함수입니다.
    ROS2 노드를 초기화하고, PyQt5 애플리케이션을 실행합니다.
    """
    rclpy.init(args=args)  # ROS2 초기화
    ros_node = TableMonitorNode()  # ROS2 노드 생성

    app = QApplication(sys.argv)  # PyQt5 애플리케이션 생성
    main_window = MainWindow(ros_node)  # 메인 윈도우 생성
    main_window.show()  # 윈도우 표시

    app.exec_()  # 이벤트 루프 실행
    ros_node.destroy_node()  # ROS2 노드 종료
    rclpy.shutdown()  # ROS2 종료


if __name__ == "__main__":
    main()  # 프로그램 실행
