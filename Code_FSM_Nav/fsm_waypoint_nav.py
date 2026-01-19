import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
import math


def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class FSMNavigator(Node):

    def __init__(self):
        super().__init__('fsm_waypoint_nav')

        # FSM States
        self.IDLE = 0
        self.ROTATE = 1
        self.MOVE = 2
        self.NEXT = 3
        self.DONE = 4

        self.state = self.IDLE

        # Robot pose
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        # Waypoints (EDIT THIS)
        self.waypoints = [
            (2.7, 0),
            (2.7, 0.8),
            (4.5, 0.8),
            (4.5, 2),
            (5.0, 4.9),
            (5.5, 4.8),
            (6.0, 4.8),
            (7.0, 4.6),
            (8.0, 4.2),
            (8.5, 3.9),
            (8.5, 3.4),
            (8.1, 2.7),

        ]

        self.current_wp = 0

        # Control gains
        self.Kp_ang = 1.5
        self.Kp_lin = 0.5

        # Tolerances
        self.angle_tol = 0.05
        self.dist_tol = 0.15

        # ROS Interfaces
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/diff_cont/odom', self.odom_callback, 10)

        self.timer = self.create_timer(0.05, self.fsm_loop)

        self.get_logger().info("FSM Waypoint Navigator Started")

    # ---------------- ODOM ----------------
    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny, cosy)

    # ---------------- FSM LOOP ----------------
    def fsm_loop(self):

        cmd = Twist()

        if self.state == self.IDLE:
            if self.current_wp < len(self.waypoints):
                self.get_logger().info(f"Going to waypoint {self.current_wp+1}")
                self.state = self.ROTATE
            else:
                self.state = self.DONE

        elif self.state == self.ROTATE:
            xg, yg = self.waypoints[self.current_wp]
            angle_to_goal = math.atan2(yg - self.y, xg - self.x)
            error = normalize_angle(angle_to_goal - self.yaw)

            cmd.angular.z = self.Kp_ang * error

            if abs(error) < self.angle_tol:
                self.state = self.MOVE

        elif self.state == self.MOVE:
            xg, yg = self.waypoints[self.current_wp]
            dx = xg - self.x
            dy = yg - self.y

            dist = math.sqrt(dx*dx + dy*dy)
            angle_to_goal = math.atan2(dy, dx)
            error = normalize_angle(angle_to_goal - self.yaw)

            cmd.linear.x = self.Kp_lin * dist
            cmd.angular.z = self.Kp_ang * error

            if dist < self.dist_tol:
                self.state = self.NEXT

        elif self.state == self.NEXT:
            self.current_wp += 1
            self.state = self.IDLE

        elif self.state == self.DONE:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.get_logger().info("All waypoints reached")
            self.cmd_pub.publish(cmd)
            return

        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = FSMNavigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

