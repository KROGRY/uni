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
        
        self.state = "navigating"  # Possible states: "navigating", "avoiding_obstacle"
        self.avoidance_direction = None  # "left" or "right"
        
        self.waypoints = [
            # Waypoints here
        ]

        self.current_waypoint_index = 0
        self.rate = rospy.Rate(10)  # 10Hz
        
    def odom_callback(self, msg):
        # Odom callback as before
        
    def lidar_callback(self, msg):
        # Lidar callback as before but also determine state
        
    def calculate_angle_to_target(self, target):
        # Angle calculation as before

    def navigate_to_waypoint(self):
        if self.current_position is None or self.current_waypoint_index >= len(self.waypoints):
            return

        target = self.waypoints[self.current_waypoint_index]
        target_angle = self.calculate_angle_to_target(target)
        angle_diff = target_angle - self.current_orientation

        distance_to_target = math.sqrt((target[0] - self.current_position.x) ** 2 + (target[1] - self.current_position.y) ** 2)

        twist = Twist()

        # State-based decision making
        if self.state == "avoiding_obstacle":
            # Adjust this threshold as necessary for your environment
            if self.obstacle_distance > 0.7:  # Considered safe distance
                self.state = "navigating"  # Switch back to navigating state
            else:
                twist.linear.x = 0.0
                twist.angular.z = 0.5 if self.avoidance_direction == "left" else -0.5
        elif self.state == "navigating":
            if self.obstacle_distance < 0.5:  # Obstacle too close
                self.state = "avoiding_obstacle"
                # Determine direction based on obstacle location
                self.avoidance_direction = "right" if self.left_side_obstacle_distance < self.right_side_obstacle_distance else "left"
                twist.linear.x = 0.0
                twist.angular.z = 0.5 if self.avoidance_direction == "left" else -0.5
            elif distance_to_target > 0.2:
                if abs(angle_diff) > 0.1:
                    twist.angular.z = 0.3 if angle_diff > 0 else -0.3
                else:
                    twist.linear.x = 0.5
            else:
                self.current_waypoint_index += 1
                print("Waypoint reached, moving to next.")
        else:
            print("Unknown state:", self.state)

        self.movement_pub.publish(twist)

    def run(self):
        while not rospy.is_shutdown():
            self.navigate_to_waypoint()
            self.rate.sleep()

if __name__ == '__main__':
    try:
        navigator = RoomNavigator()
        navigator.run()
    except rospy.ROSInterruptException:
        pass
