import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32, Bool, Float32MultiArray
import math
import time

BOX_LOCATION = [-0.33]
ROTATIOM_LOCATION = [-0.03]
GOALS = {
    1: [-0.05],
    2: [-0.23],
    3: [-0.42],
}

LINEAR_SPEED = 0.1
DISTANCE_THRESHOLD = 0.01


class TurtleBotController(Node):
    def __init__(self):
        super().__init__('turtlebot_controller')

        self.aruco_subscription = self.create_subscription(
            Float32MultiArray, 'aruco_marker_pose', self.aruco_callback, 10)
        self.goal_subscriber = self.create_subscription(
            Int32, 'goal', self.goal_callback, 10)
        self.done_subscriber = self.create_subscription(
            Int32, 'done', self.done_callback, 10)
        self.grab_publisher = self.create_publisher(Int32, 'grab_box', 10)
        self.step_publisher = self.create_publisher(Int32, 'step', 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.current_pose = None
        self.current_goal = None
        self.state = 'IDLE'
        self.target_pose = None

        self.timer = self.create_timer(0.1, self.move_to_goal)
    
    def publish_step(self, value):
        msg = Int32()
        msg.data = value
        self.step_publisher.publish(msg)
        self.get_logger().info(f"Published: {value}")

    def aruco_callback(self, msg):
        if len(msg.data) < 6:
            self.get_logger().warning("Invalid ArUco marker data received.")
            return
        self.current_pose = msg.data

    def goal_callback(self, msg):
        goal_id = msg.data
        if goal_id in GOALS:
            self.current_goal = GOALS[goal_id]
            self.get_logger().info(f"New goal received: Goal {goal_id}")
            self.state = 'PICKUP'
            self.target_pose = self.current_pose
        else:
            self.get_logger().warning(f"Invalid goal ID: {goal_id}")

    def done_callback(self, msg):
        if self.state == 'WAIT_PICKUP' and msg.data == 0:
            self.get_logger().info("Box has been grabbed. Moving to box location.")
            self.state = 'BOX'
            self.target_pose = BOX_LOCATION
            self.publish_step(2)
        elif self.state == 'WAIT_BOX' and msg.data == 1:
            self.get_logger().info("Box has been dropped. Moving to goal location.")
            self.state = 'ROTATION'
            self.target_pose = ROTATIOM_LOCATION
        elif self.state == 'WAIT_ROTATION' and msg.data == 2:
            self.get_logger().info("Box has been dropped. Moving to goal location.")
            self.state = 'GOAL'
            self.target_pose = self.current_goal

    def publish_grab_command(self, grab):
        msg = Int32()
        msg.data = grab
        self.grab_publisher.publish(msg)
        if grab == 0:
            self.get_logger().info("Published grab_box: 0 (Pickup)")
        elif grab == 1:
            self.get_logger().info("Published grab_box: 1 (Rotation)")
        elif grab == 2:
            self.get_logger().info("Published grab_box: 2 (Drop)")

    def move_to_goal(self):
        if self.state in ['WAIT_PICKUP', 'WAIT_BOX']:
            return

        if self.current_pose is None or self.target_pose is None:
            return

        distance = self.target_pose[0] - self.current_pose[0]
        
        twist = Twist()

        if abs(distance) < DISTANCE_THRESHOLD:
            if self.state == 'PICKUP':
                self.get_logger().info("Reached pickup location. Waiting for pickup completion...")
                self.state = 'WAIT_PICKUP'
            elif self.state == 'BOX':
                self.get_logger().info("Reached box location. Waiting to drop box...")
                self.state = 'WAIT_BOX'
                time.sleep(30)
                self.publish_grab_command(0)
            elif self.state == 'ROTATION':
                self.get_logger().info("Rotated box location. Waiting to rotate box...")
                self.state = 'WAIT_ROTATION'
                self.publish_grab_command(1)
            elif self.state == 'GOAL':
                self.get_logger().info("Reached final goal. Task completed!")
                self.publish_grab_command(2)
                self.state = 'IDLE'
            return

        twist.linear.x = LINEAR_SPEED if distance > 0 else -LINEAR_SPEED

        self.cmd_vel_publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    controller = TurtleBotController()
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
