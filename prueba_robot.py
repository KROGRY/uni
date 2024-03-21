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
        
        self.waypoints = [
            (2.00, 5.50),  # Punto 1
            (3.50, 4.50),  # Punto 2
            (3.50, 6.50),  # Punto 3
            (6.75, 5.50),  # Punto 4
            (6.75, 1.50),  # Punto 5
            (5.00, 1.50),  # Punto 6
            (5.00, 2.50),  # Punto 7
            (2.50, 3.75),  # Punto 8
            (2.50, 0.25),  # Punto 9
            (0.50, 4.50)   # Punto 10
        ]

        self.current_waypoint_index = 0
        self.reached_waypoints = []

        self.rate = rospy.Rate(10)  # 10Hz
        
    def odom_callback(self, msg):
        self.current_position = msg.pose.pose.position
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        self.current_orientation = euler_from_quaternion(orientation_list)[2]  # yaw
        
    def lidar_callback(self, msg):
        # Original code to find the minimum distance
        self.obstacle_distance = min(min(msg.ranges), 10.0)  # Limiting range to 10 meters

        # New code to determine if obstacles are more on the left or right
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
        if self.obstacle_distance < 0.5:  # If obstacle too close
            twist.linear.x = 0.0
            # Decide turning direction based on obstacle location
            if self.left_side_obstacle_distance < self.right_side_obstacle_distance:
                twist.angular.z = -0.5  # Turn right
            else:
                twist.angular.z = 0.5  # Turn left
        elif distance_to_target > 0.2:
            if abs(angle_diff) > 0.1:
                twist.angular.z = 0.3 if angle_diff > 0 else -0.3
            else:
                twist.linear.x = 0.5
        else:
            self.current_waypoint_index += 1
            print("Waypoint reached, moving to next.")

        self.movement_pub.publish(twist)

if __name__ == '__main__':
    try:
        navigator = RoomNavigator()
        navigator.run()
    except rospy.ROSInterruptException:
        pass
