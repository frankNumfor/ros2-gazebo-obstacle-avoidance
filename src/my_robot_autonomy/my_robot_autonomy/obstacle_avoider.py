import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class ObstacleAvoider(Node):
    """Simple lidar-based reactive obstacle avoidance.

    Drives straight at full speed. Only when an obstacle enters the forward
    cone within stop_distance does it commit to a turn direction (toward
    whichever side has more clearance) and rotate in place until the front
    is clear again past resume_distance. The turn direction is picked once
    per avoidance episode, not recomputed every scan, so it can't oscillate.
    resume_distance > stop_distance gives hysteresis so it doesn't flicker
    right at the trigger boundary.
    """

    def __init__(self):
        super().__init__('obstacle_avoider')

        self.declare_parameter('linear_speed', 0.7)
        self.declare_parameter('angular_speed', 2.0)
        self.declare_parameter('front_half_angle_deg', 45.0)
        self.declare_parameter('stop_distance', 1.0)
        self.declare_parameter('resume_distance', 1.4)

        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.front_half_angle = np.radians(self.get_parameter('front_half_angle_deg').value)
        self.stop_distance = self.get_parameter('stop_distance').value
        self.resume_distance = self.get_parameter('resume_distance').value

        self.avoiding = False
        self.turn_direction = 1.0

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, qos_profile_sensor_data)

    def scan_callback(self, msg):
        ranges = np.array(msg.ranges)
        ranges = np.where(np.isfinite(ranges), ranges, msg.range_max)
        angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment

        front = ranges[np.abs(angles) <= self.front_half_angle]
        left = ranges[(angles > 0) & (angles <= self.front_half_angle)]
        right = ranges[(angles < 0) & (angles >= -self.front_half_angle)]

        front_min = front.min() if front.size else msg.range_max
        left_min = left.min() if left.size else msg.range_max
        right_min = right.min() if right.size else msg.range_max

        cmd = Twist()

        if not self.avoiding and front_min < self.stop_distance:
            self.avoiding = True
            self.turn_direction = 1.0 if left_min > right_min else -1.0

        if self.avoiding:
            cmd.angular.z = self.angular_speed * self.turn_direction
            if front_min > self.resume_distance:
                self.avoiding = False
        else:
            cmd.linear.x = self.linear_speed

        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoider()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
