import rospy
import math
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

class RoomNavigator:
    def __init__(self):
        rospy.init_node('room_navigator', anonymous=True)
        
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.lidar_sub = rospy.Subscriber('/scan', LaserScan, self.lidar_callback)
        self.movement_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        
        self.current_position = None
        self.current_orientation = None
        self.obstacle_distance = float('inf')
        
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

        self.rate = rospy.Rate(10)  # 10Hz
        
    def odom_callback(self, msg):
        self.current_position = msg.pose.pose.position
        self.current_orientation = msg.pose.pose.orientation
        
    def lidar_callback(self, msg):
        # Simple obstacle detection
        self.obstacle_distance = min(min(msg.ranges[300:420]), 10.0)  # Limiting range to 10 meters
    
    def navigate_to_waypoint(self):
        twist = Twist()
        
        if self.current_waypoint_index >= len(self.waypoints):
            print("Reached all waypoints.")
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.movement_pub.publish(twist)
            rospy.signal_shutdown("Navigation complete")
            return

        # Placeholder for actual navigation logic to waypoint
        # Here you'd include logic to orient towards and move to the next waypoint, 
        # adjusting for obstacles as detected in lidar_callback.
        
        # For simplicity, this example will just move forward slowly unless an obstacle is detected.
        if self.obstacle_distance < 1.0:  # 1 meter threshold for obstacles
            print("Obstacle detected! Stopping.")
            twist.linear.x = 0.0
            twist.angular.z = 0.5  # Turn slowly in place
        else:
            print("Moving towards waypoint.")
            twist.linear.x = 0.2
            twist.angular.z = 0.0
        
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
