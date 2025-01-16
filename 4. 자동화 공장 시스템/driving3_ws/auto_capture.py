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
from std_msgs.msg import Bool,String
import math
from control_msgs.action import GripperCommand
from rclpy.action import ActionClient
import random
import cv2

# 비디오 파일 경로와 저장 경로 설정
output_dir = 'frames'  # 프레임을 저장할 디렉토리

# 저장 디렉토리가 존재하지 않으면 생성
if not os.path.exists(output_dir):
    os.makedirs(output_dir)

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

usage = """
Control Your OpenManipulator!
---------------------------
Joint Space Control:
- Joint1 : Increase (Y), Decrease (H)
- Joint2 : Increase (U), Decrease (J)
- Joint3 : Increase (I), Decrease (K)
- Joint4 : Increase (O), Decrease (L)

INIT : (1)

CTRL-C to quit
"""

joint_angle_delta = 0.05  # radian


class Turtlebot3ManipulationTest(Node): 
    # settings = None
    # if os.name != 'nt':
    # 	settings = termios.tcgetattr(sys.stdin)
        
    def __init__(self): 

        super().__init__('turtlebot3_manipulation_test')
        key_value = ''
        self.cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.joint_pub = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.gripper_action_client = ActionClient(self, GripperCommand, 'gripper_controller/gripper_cmd')
        self.auto_capture = self.create_subscription(Bool, 'capture',self.capture_callback,10)
        # self.timer = self.create_timer(1.0, self.timer_callback)
        # Twist is geometry_msgs for linear and angular velocity 
        self.move_cmd = Twist() 
        # Linear speed in x in meters/second is + (forward) or 
        #    - (backwards) 
        self.move_cmd.linear.x = 1.3   # Modify this value to change speed 
        # Turn at 0 radians/s 
        self.move_cmd.angular.z = 0.8 
        # Modify this value to cause rotation rad/s 

        self.trajectory_msg = JointTrajectory()
        current_time = self.get_clock().now()
        self.trajectory_msg.header = Header()
        # self.trajectory_msg.header.stamp = current_time.to_msg()
        self.trajectory_msg.header.frame_id = ''
        self.trajectory_msg.joint_names = ['joint1', 'joint2', 'joint3', 'joint4']

        point = JointTrajectoryPoint()
        # point.positions = [0.003, math.pi / 4.0, -0.489, 2.041]
        point.positions = [0.0] * 4
        point.velocities = [0.0] * 4
        point.time_from_start.sec = 3
        point.time_from_start.nanosec = 0
        self.cap_active = False  # 상태 추적
        self.cap = cv2.VideoCapture('/dev/video0')
        if not self.cap.isOpened():
            self.get_logger().info("카메라를 열수 없습니다.")

        self.saved_frame_count = 0

        self.trajectory_msg.points = [point]
        self.timer = self.create_timer(0.1, self.timer_callback)

        # self.joint_pub.publish(self.trajectory_msg)
    def capture_callback(self, msg):
        if msg.data:  # step = 1
            if not self.cap_active:  # 이미 활성화 상태가 아니면 타이머 시작
                self.cap_active = True
                self.cap_timer = self.create_timer(5, self.cap_timer_callback)
                self.get_logger().info("Capture started.")
        else:  # step = 0
            if self.cap_active:  # 활성화 상태라면 타이머 중지
                self.cap_active = False
                if self.cap_timer is not None:
                    self.cap_timer.cancel()
                    self.cap_timer = None
                self.get_logger().info("Capture stopped.")

    def cap_timer_callback(self):
        tutorial_x = random.randint(100,200)
        tutorial_y = random.randint(-100,100)
        tutorial_z = random.randint(0,200)
        try:
            J0, J1, J2, J3, sxy, sr1, sr2, sr3, St, Rt = solv_robot_arm2(tutorial_x, tutorial_y, tutorial_z, r1, r2, r3)
            self.trajectory_msg.points[0].positions[0] = sxy
            self.trajectory_msg.points[0].positions[1] = sr1 + th1_offset
            self.trajectory_msg.points[0].positions[2] = sr2 + th2_offset
            self.trajectory_msg.points[0].positions[3] = sr3
        except Exception as e:
            print(e)
        ret, frame = self.cap.read()
        # 비디오가 끝났을 경우 반복 종료
        if not self.cap.isOpened():
            self.get_logger().info("카메라를 열수 없습니다.")
        if not ret:
            self.get_logger().error("Camera not available!")
            return
        frame_filename = os.path.join(output_dir, f'frame_{self.saved_frame_count:04d}.jpg')
        cv2.imwrite(frame_filename, frame)
        self.saved_frame_count += 1
        print(f'Saved: {frame_filename}')
        
    def send_gripper_goal(self, position):
        # """Gripper goal 설정"""
        goal = GripperCommand.Goal()
        goal.command.position = position
        goal.command.max_effort = -1.0

        if not self.gripper_action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error("Gripper action server not available!")
            return

        self.gripper_action_client.send_goal_async(goal)
        
    def timer_callback(self):
        self.joint_pub.publish(self.trajectory_msg)
        # self.cmd_vel.publish(self.move_cmd) 

def spin_node(node):
    while rclpy.ok():
        rclpy.spin_once(node)

node = None

def main(args=None):
    # rclpy.init(args=args)
    tutorial_x = 200
    tutorial_y = 0
    tutorial_z = 100
    try:
        rclpy.init()
    except Exception as e:
        print(e)

    try:
        node = Turtlebot3ManipulationTest()
    except Exception as e:
        print(e)
    # 노드를 멀티쓰레드로 실행
    node_thread = threading.Thread(target=spin_node, args=(node,))
    node_thread.start()

    try:
        while(rclpy.ok()):
            key_value = getkey.getkey()
            
            if key_value == '1':
                node.trajectory_msg.points[0].positions = [0.0] * 4
                # node.joint_pub.publish(node.trajectory_msg)
                print('joint1 +')
            elif key_value == 'y':
                node.trajectory_msg.points[0].positions[0] += joint_angle_delta
                # node.joint_pub.publish(node.trajectory_msg)
                print('joint1 +')
            elif key_value == 'h':
                node.trajectory_msg.points[0].positions[0] -= joint_angle_delta
                # node.joint_pub.publish(node.trajectory_msg)
                print('joint1 -')
            elif key_value == 'u':
                node.trajectory_msg.points[0].positions[1] += joint_angle_delta
                #node.joint_pub.publish(node.trajectory_msg)
                print('joint2 +')
            elif key_value == 'j':
                node.trajectory_msg.points[0].positions[1] -= joint_angle_delta
                #node.joint_pub.publish(node.trajectory_msg)
                print('joint2 -')
            elif key_value == 'i':
                node.trajectory_msg.points[0].positions[2] += joint_angle_delta
                #node.joint_pub.publish(node.trajectory_msg)
                print('joint3 +')
            elif key_value == 'k':
                node.trajectory_msg.points[0].positions[2] -= joint_angle_delta
                #node.joint_pub.publish(node.trajectory_msg)
                print('joint3 -')
            elif key_value == 'o':
                node.trajectory_msg.points[0].positions[3] += joint_angle_delta
                #node.joint_pub.publish(node.trajectory_msg)
                print('joint4 +')
            elif key_value == 'l':
                node.trajectory_msg.points[0].positions[3] -= joint_angle_delta
                #node.joint_pub.publish(node.trajectory_msg)
                print('joint4 -')
            elif key_value == 'z':
                
                J0, J1, J2, J3, sxy, sr1, sr2, sr3, St, Rt = solv_robot_arm2(tutorial_x, tutorial_y, tutorial_z, r1, r2, r3)

                node.trajectory_msg.points[0].positions[0] = sxy
                node.trajectory_msg.points[0].positions[1] = sr1 + th1_offset
                node.trajectory_msg.points[0].positions[2] = sr2 + th2_offset
                node.trajectory_msg.points[0].positions[3] = sr3
                #node.joint_pub.publish(node.trajectory_msg)

            elif key_value == 'x':
                tutorial_x -= 10
                
                J0, J1, J2, J3, sxy, sr1, sr2, sr3, St, Rt = solv_robot_arm2(tutorial_x, tutorial_y, tutorial_z, r1, r2, r3)
                node.trajectory_msg.points[0].positions[0] = sxy
                node.trajectory_msg.points[0].positions[1] = sr1 + th1_offset
                node.trajectory_msg.points[0].positions[2] = sr2 + th2_offset
                node.trajectory_msg.points[0].positions[3] = sr3
                #node.joint_pub.publish(node.trajectory_msg)

            elif key_value == 'w':
                tutorial_x += 10
                
                J0, J1, J2, J3, sxy, sr1, sr2, sr3, St, Rt = solv_robot_arm2(tutorial_x, tutorial_y, tutorial_z, r1, r2, r3)
                node.trajectory_msg.points[0].positions[0] = sxy
                node.trajectory_msg.points[0].positions[1] = sr1 + th1_offset
                node.trajectory_msg.points[0].positions[2] = sr2 + th2_offset
                node.trajectory_msg.points[0].positions[3] = sr3
                #node.joint_pub.publish(node.trajectory_msg)

            elif key_value == 'a':
                tutorial_y += 10
                
                J0, J1, J2, J3, sxy, sr1, sr2, sr3, St, Rt = solv_robot_arm2(tutorial_x, tutorial_y, tutorial_z, r1, r2, r3)
                node.trajectory_msg.points[0].positions[0] = sxy
                node.trajectory_msg.points[0].positions[1] = sr1 + th1_offset
                node.trajectory_msg.points[0].positions[2] = sr2 + th2_offset
                node.trajectory_msg.points[0].positions[3] = sr3
                #node.joint_pub.publish(node.trajectory_msg)

            elif key_value == 'd':
                tutorial_y -= 10
                
                J0, J1, J2, J3, sxy, sr1, sr2, sr3, St, Rt = solv_robot_arm2(tutorial_x, tutorial_y, tutorial_z, r1, r2, r3)
                node.trajectory_msg.points[0].positions[0] = sxy
                node.trajectory_msg.points[0].positions[1] = sr1 + th1_offset
                node.trajectory_msg.points[0].positions[2] = sr2 + th2_offset
                node.trajectory_msg.points[0].positions[3] = sr3
                #node.joint_pub.publish(node.trajectory_msg)

            elif key_value == 'e':
                tutorial_z += 10
                
                J0, J1, J2, J3, sxy, sr1, sr2, sr3, St, Rt = solv_robot_arm2(tutorial_x, tutorial_y, tutorial_z, r1, r2, r3)
                node.trajectory_msg.points[0].positions[0] = sxy
                node.trajectory_msg.points[0].positions[1] = sr1 + th1_offset
                node.trajectory_msg.points[0].positions[2] = sr2 + th2_offset
                node.trajectory_msg.points[0].positions[3] = sr3
                #node.joint_pub.publish(node.trajectory_msg)

            elif key_value == 'c':
                tutorial_z -= 10
                
                J0, J1, J2, J3, sxy, sr1, sr2, sr3, St, Rt = solv_robot_arm2(tutorial_x, tutorial_y, tutorial_z, r1, r2, r3)
                node.trajectory_msg.points[0].positions[0] = sxy
                node.trajectory_msg.points[0].positions[1] = sr1 + th1_offset
                node.trajectory_msg.points[0].positions[2] = sr2 + th2_offset
                node.trajectory_msg.points[0].positions[3] = sr3
                #node.joint_pub.publish(node.trajectory_msg)

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
        node.cap.release()
        cv2.destroyAllWindows()
        node_thread.join()


if __name__== "__main__": 
    main() 