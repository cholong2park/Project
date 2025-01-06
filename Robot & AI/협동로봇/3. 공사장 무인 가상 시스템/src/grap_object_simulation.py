#!/usr/bin/env python 
# Set linear and angular values of Turtlesim's speed and turning. 

import os
import select
import sys
import getkey
import threading

import rclpy	# Needed to create a ROS node 
from geometry_msgs.msg import Twist    # Message that moves base
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Header
from rclpy.node import Node
from std_msgs.msg import Bool,Int32MultiArray, Int32
from sensor_msgs.msg import Image
import math
from control_msgs.action import GripperCommand
from rclpy.action import ActionClient
import random
import cv2
from ultralytics import YOLO  # YOLOv8 패키지 import
import time
from cv_bridge import CvBridge
from rclpy.executors import MultiThreadedExecutor

r1 = 130
r2 = 124
r3 = 126
th1_offset = - math.atan2(0.024, 0.128)
th2_offset = - 0.5*math.pi - th1_offset
def solv2(r1, r2, r3):
    d1 = (r3**2 - r2**2 + r1**2) / (2*r3)
    d2 = (r3**2 + r2**2 - r1**2) / (2*r3)

    s1 = math.acos(d1 / r1)
    s2 = math.acos(d2 / r2)

    return s1, s2

def solv_robot_arm2(x, y, z, r1, r2, r3):
    Rt = math.sqrt(x**2 + y**2 + z**2)
    Rxy = math.sqrt(x**2 + y**2)
    St = math.asin(z / Rt)
    #   Sxy = math.acos(x / Rxy)
    Sxy = math.atan2(y, x)

    s1, s2 = solv2(r1, r2, Rt)

    sr1 = math.pi/2 - (s1 + St)
    sr2 = s1 + s2
    sr2_ = sr1 + sr2
    sr3 = math.pi - sr2_

    J0 = (0, 0, 0)
    J1 = (J0[0] + r1 * math.sin(sr1)  * math.cos(Sxy),
        J0[1] + r1 * math.sin(sr1)  * math.sin(Sxy),
        J0[2] + r1 * math.cos(sr1))
    J2 = (J1[0] + r2 * math.sin(sr1 + sr2) * math.cos(Sxy),
        J1[1] + r2 * math.sin(sr1 + sr2) * math.sin(Sxy),
        J1[2] + r2 * math.cos(sr1 + sr2))
    J3 = (J2[0] + r3 * math.sin(sr1 + sr2 + sr3) * math.cos(Sxy),
        J2[1] + r3 * math.sin(sr1 + sr2 + sr3) * math.sin(Sxy),
        J2[2] + r3 * math.cos(sr1 + sr2 + sr3))

    return J0, J1, J2, J3, Sxy, sr1, sr2, sr3, St, Rt

joint_angle_delta = 0.05  # radian
class image(Node):
    def __init__(self): 
        super().__init__('image_subscriber')
        self.image_sub = self.create_subscription(Image, '/pi_camera/image_raw', self.image_callback, 10)
        self.bridge = CvBridge()
    def image_callback(self, msg):
        try:
            self.cap = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.get_logger().info(f"이미지 최신화")
        except Exception as e:
            self.get_logger().error(f"Failed to process image: {e}")

class Turtlebot3ManipulationTest(Node): 
    # settings = None
    # if os.name != 'nt':
    # 	settings = termios.tcgetattr(sys.stdin)
        
    def __init__(self,image_thread): 
        super().__init__('turtlebot3_manipulation_test')
        self.image_thread = image_thread
        self.cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.joint_pub = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.gripper_action_client = ActionClient(self, GripperCommand, 'gripper_controller/gripper_cmd')
        self.grap_object_sub = self.create_subscription(Int32MultiArray,'task',self.grap_callback,10)
        self.done_pub = self.create_publisher(Int32, 'done', 10)

        self.trajectory_msg = JointTrajectory()
        self.trajectory_msg.header = Header()
        self.trajectory_msg.header.frame_id = ''
        self.trajectory_msg.joint_names = ['joint1', 'joint2', 'joint3', 'joint4']
        self.x, self.y, self.z = -150, 0, 190
        point = JointTrajectoryPoint()
        self.trajectory_msg.points = [point]
        self.calculate_publish()
        self.send_gripper_goal(0.025)  # Open
        self.model = YOLO('best.pt')  # YOLOv8 모델 로드
        

    def calculate_publish(self):
        try:
            J0, J1, J2, J3, sxy, sr1, sr2, sr3, St, Rt = solv_robot_arm2(self.x, self.y, self.z, r1, r2, r3)
            self.trajectory_msg.points[0].positions = [sxy, sr1 + th1_offset, sr2 + th2_offset, sr3]
        except Exception as e:
            print(e)
            self.x, self.y, self.z = -150, 0, 150
            self.calculate_publish()
            return
        self.joint_pub.publish(self.trajectory_msg)
        self.get_logger().info(f"x: {self.x}, y: {self.y}, z: {self.z}")

    def grap_callback(self,msg):
        red, blue = msg.data
        self.get_logger().info(f"Grap started. red: {red}, blue: {blue}")
        for _ in range(red):
            self.grap('red')
        done = Int32()
        done.data = 0  # True 값 설정
        self.done_pub.publish(done)  # 퍼블리시

    def grap(self,color):
        self.x, self.y, self.z = -150, 0, 150
        self.get_logger().info(f"Starting grap process with target color: {color}")
        self.calculate_publish()
        while True:
            frame = self.image_thread.cap
            results = self.model(frame)  # YOLOv8 모델로 프레임 처리
            detections = results[0].boxes            
            objects = []
            self.get_logger().info(f"Detected {len(detections)} objects in the frame.")
            if 70<self.z:
                diff=10
            else:
                self.grip_and_release()
                break
            for box in detections:
                label = self.model.names[int(box.cls[0])]  # 클래스 라벨
                if label == color:
                    objects.append(box)
            # 가장 왼쪽에 있는 'red' 객체 선택
            if objects:
                left = min(objects, key=lambda box: box.xyxy[0][0])  # x_min 기준으로 정렬
                x_min = int(left.xyxy[0][0].item())  # 좌표를 숫자(float)로 변환
                x_max = int(left.xyxy[0][2].item())
                y_min = int(left.xyxy[0][1].item())
                y_max = int(left.xyxy[0][3].item())

                # 가로 길이와 세로 길이를 계산
                width = x_max - x_min
                height = y_max - y_min
                area = width * height
                self.get_logger().info(f"Object area: {area}, width: {width}, height: {height}")
                x_center = int((x_max+x_min) / 2)
                y_center = int((y_max+y_min) / 2)
                frame_center = (320, 240)
                # 화면 중심과 객체 중심의 오차 계산
                error_x = x_center - frame_center[0]
                error_y = y_center - frame_center[1]   # 화면 좌표계는 y가 아래로 증가
                self.get_logger().info(f"Object center: ({x_center}, {y_center}), Error: ({error_x}, {error_y})")

                # 로봇팔 제어를 위한 새로운 좌표 설정
                if abs(error_x)>50:
                    self.y += error_x/abs(error_x) * diff  # 비례 상수 조정
                elif abs(error_y)>50:
                    self.x -= error_y/abs(error_y) * diff  # 비례 상수 조정
                else:
                    self.z -= 30  # 비례 상수 조정

                self.calculate_publish()

            time.sleep(0.3)

    def grip_and_release(self):
        self.x+=60
        self.z-=35
        if self.y>0:
            self.y+=20
        else:
            self.y-=20
        self.calculate_publish()
        time.sleep(1)
        self.send_gripper_goal(-0.015)  # Close
        time.sleep(1)
        self.z+=55
        self.calculate_publish()
        time.sleep(1)
        self.x, self.y, self.z = 0, -220, 100
        self.calculate_publish()
        time.sleep(0.5)
        self.send_gripper_goal(0.025)  # Open
        time.sleep(1)
        
    def send_gripper_goal(self, position):
        # """Gripper goal 설정"""
        goal = GripperCommand.Goal()
        goal.command.position = position
        goal.command.max_effort = -1.0

        if not self.gripper_action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error("Gripper action server not available!")
            return

        self.gripper_action_client.send_goal_async(goal)

def spin_node(node):
    while rclpy.ok():
        rclpy.spin_once(node)

node = None

def main(args=None):
    # rclpy.init(args=args)

    rclpy.init()
    image_subscriber = image()

    try:
        node = Turtlebot3ManipulationTest(image_subscriber)
    except Exception as e:
        print(e)
    # 노드를 멀티쓰레드로 실행
    executor = MultiThreadedExecutor()

    # 노드 추가
    executor.add_node(image_subscriber)
    executor.add_node(node)
    executor.spin()

    try:
        while(rclpy.ok()):
            key_value = getkey.getkey()
            if key_value == 'x':
                node.x -= 10
                node.calculate_publish()
            elif key_value == 'w':
                node.x += 10
                node.calculate_publish()
            elif key_value == 'a':
                node.y += 10
                node.calculate_publish()
            elif key_value == 'd':
                node.y -= 10
                node.calculate_publish()
            elif key_value == 'e':
                node.z += 10
                node.calculate_publish()
            elif key_value == 'c':
                node.z -= 10
                node.calculate_publish()
            elif key_value == 'r':
                node.send_gripper_goal(0.025)  # Open

            elif key_value == 't':
                node.send_gripper_goal(-0.015)  # Close

            elif key_value == 'q':
                break

    except Exception as e:
        print(e)

    finally:
        # if os.name != 'nt':
        #     termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__== "__main__": 
    main() 