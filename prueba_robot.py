import rospy
import math
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion

class RoomNavigator:
    def __init__(self):
        rospy.init_node('room_navigator', anonymous=True)

        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.lidar_sub = rospy.Subscriber('/scan', LaserScan, self.lidar_callback)
        self.movement_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

        self.current_position = None
        self.current_orientation = None
        self.obstacle_distance = float('inf')
        self.left_side_obstacle_distance = float('inf')
        self.right_side_obstacle_distance = float('inf')

        self.state = "navigating"
        self.avoidance_direction = None

        self.waypoints = [
            (1.00, 6.2),  # Punto 1
            (2.00, 5.50),  # Punto 1
            (3.50, 4.50),  # Punto 2
            (3.50, 6.50),  # Punto 3
            (3.50, 4.50),  # Punto 3
            (6, 4.50),    # Punto 3
            (6, 5.5),     # Punto 3
            (6.75, 5.50),  # Punto 4
            (6.75, 6.50),  # Punto 4
            (7.5, 6.50),   # Punto 4
            (7.5, 1.50),   # Punto 4
            (6.75, 1.50),  # Punto 5
            (5.00, 1.50),  # Punto 6
            (5.00, 2.50),  # Punto 7
            (4.2, 2.50),   # Punto 7
            (4.2, 3.50),   # Punto 7
            (2.50, 3.55),  # Punto 8
            (4.2, 3.50),   # Punto 7
            (4.2, 0.5),    # Punto 7
            (2.50, 0.5),   # Punto 9
            (0.5, 0.5),    # Punto 9
            (0.50, 4.50)   # Punto 10
        ]
        self.current_waypoint_index = 0
        self.rate = rospy.Rate(10)  # 10Hz

    def odom_callback(self, msg):
        self.current_position = msg.pose.pose.position
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        self.current_orientation = euler_from_quaternion(orientation_list)[2]

    def lidar_callback(self, msg):
        self.obstacle_distance = min(min(msg.ranges), 10.0)

        mid_index = len(msg.ranges) // 2
        left_ranges = msg.ranges[mid_index:]
        right_ranges = msg.ranges[:mid_index]

        self.left_side_obstacle_distance = sum(left_ranges) / len(left_ranges)
        self.right_side_obstacle_distance = sum(right_ranges) / len(right_ranges)

    def calculate_angle_to_target(self, target):
        dx = target[0] - self.current_position.x
        dy = target[1] - self.current_position.y
        return math.atan2(dy, dx)

    def navigate_to_waypoint(self):
        if self.current_position is None or self.current_waypoint_index >= len(self.waypoints):
            return

        target = self.waypoints[self.current_waypoint_index]
        target_angle = self.calculate_angle_to_target(target)
        angle_diff = target_angle - self.current_orientation

        distance_to_target = math.sqrt((target[0] - self.current_position.x) ** 2 + (target[1] - self.current_position.y) ** 2)

        twist = Twist()

        if self.state == "avoiding_obstacle":
            if self.obstacle_distance > 0.4:  # Safe to navigate
                self.state = "navigating"
            else:
                twist.linear.x = 0.0
                twist.angular.z = 0.1 if self.avoidance_direction == "left" else -0.1
        elif self.state == "navigating":
            if self.obstacle_distance < 0.3:  # Detected an obstacle close by
                self.state = "avoiding_obstacle"
                self.avoidance_direction = "right" if self.left_side_obstacle_distance < self.right_side_obstacle_distance else "left"
                twist.linear.x = 0.0
                twist.angular.z = 0.1 if self.avoidance_direction == "left" else -0.1
            elif distance_to_target > 0.2:
                if abs(angle_diff) > 0.1:
                    # Apply a smoother angular adjustment, proportional to the angle difference but capped
                    twist.angular.z = max(min(0.5, 0.3 * angle_diff), -0.5)
                else:
                    twist.linear.x = 0.2
            else:
                self.current_waypoint_index += 1
                print("Waypoint reached, moving to next.")
        else:
            print("Unknown state:", self.state)

        self.movement_pub.publish(twist)

    def run(self):
        while not rospy.is_shutdown():
            self.navigate_to_waypoint()
            print(self.state)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        navigator = RoomNavigator()
        navigator.run()
    except rospy.ROSInterruptException:
        pass
