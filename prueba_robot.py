import rospy
import math
import time
import sys
import tf
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math
from random import seed
from random import random
# seed random number generator
seed(1)


def generate_random():
    """Generate a random value between 1 and 2

    Return
    ----------
    The random value.
    """
    # generate random numbers between 0-1
    value = random()
    scaled_value = 1 + (value * (2 - 1))
    return scaled_value


def compute_distance(x1, y1, x2, y2):
    """Compute the distance between 2 points.

    Parameters
    ----------
    x1 : float
        x coordinate of the first point.
    y1 : float
        y coordinate of the first point.
    x2 : float
        x coordinate of the second point.
    y2 : float
        y coordinate of the second point.

    Return
    ----------
    The distance between between a point (x1,y1) and another point (x2,y2).
    """
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
    

def get_odom_data():
    """Get the current pose of the robot from the /odom topic

    Return
    ----------
    The position (x, y, z) and the yaw of the robot.

    """
    try:
        (trans, rot) = tf_listener.lookupTransform(parent_frame, child_frame, rospy.Time(0))
        # rotation is a list [r, p, y]
        rotation = euler_from_quaternion(rot)
    except (tf.Exception, tf.ConnectivityException, tf.LookupException):
        rospy.loginfo("TF Exception")
        return
    # return the position (x, y, z) and the yaw
    return Point(*trans), rotation[2]


def Callback(scan):
    """This function is used as the callback function when the lidar data is subscribed, and it is used to assign the 
        scan range values to 8 different sections. So, we generate 8 sections of 45 degree each to track the obstacles. 
    """
    global div_distance
    for key in div_distance.keys():
        values = []
        if key == "0":
            for x in scan.ranges[337:360]: 
                if x <= obs_threshold and x != 'inf':
                    values.append(x)
            for x in scan.ranges[0:23]: 
                if x <= obs_threshold and x != 'inf':
                    values.append(x)
        else:
            for x in scan.ranges[23 + angle_threshold*(int(key)-1) : 23 + angle_threshold*int(key)]: 
                if x <= obs_threshold and x != 'inf':
                    values.append(x)
        div_distance[key] = values


def Robot_order():
    """This function is used to scan the lidar data and assign the actions to the robot_order dictionary,the 
        robot_order["flag"] is set true when the front path(-22 to 23 (45 degree range)) is not clear to traverse.
    """
    global robot_order

    nearest = 9999999
    distance_between_region = 0
    goal = "0"
    max_destination = "4"
    max_distance = 0.0000001

    for key, value in div_distance.items():
        distance_between_region = abs(div_cost[key]-div_cost[goal])
        
        #if there're no obstacles in that region
        if not len(value):
            #checking the cheapest option
            if (distance_between_region < nearest):
                nearest = distance_between_region
                max_distance = obs_threshold
                max_destination = key

        #check if it's the clearest option
        elif(max(value) > max_distance):
            max_distance = max(value)
            max_destination = key

    #Cost Calculation
    distance_between_region = div_cost[max_destination]-div_cost[goal]

    # We update robot_order dictionary whenever the clearest path is not 0 (front)
    robot_order["flag"] =  (nearest != 0)
    robot_order["angular_velocity"] = ((distance_between_region/max(1, abs(distance_between_region)))*3)
    robot_order["sleep_time"] = ((abs(distance_between_region)*angle_threshold*math.pi)/(180*1.20))
    

def avoid_obstacle(velocity):
    """This function is used to move the robot away from the obstcle and it is called when
        obstacle is within a certain threhold distance.
        
        -----------------
        Return:
        Modified velocity message to avoid obstacle
    """
    
    global robot_order
    angular_velocity = robot_order["angular_velocity"]
    #after detecting an obstacle, the robot shall back up a bit (negative) while
    # rotating to help in case it can't perform a stationary rotation
    velocity.linear.x = -0.5
    velocity.linear.y = 0
    velocity.angular.z = angular_velocity
    return velocity


def go_to_goal(goal_x, goal_y, position, rotation, distance_to_goal):
    """Task the robot to reach a goal (x,y) using a proportional controller.
    The current pose of the robot is retrieved from /odom topic.
    Publish the message to /cmd_vel topic.
    """

    last_rotation = 0
   
    x_start = position.x
    y_start = position.y
    angle_to_goal = math.atan2(goal_y - y_start, goal_x - x_start)

    angle_diff = angle_to_goal - rotation
    
    # proportional control to move the robot forward
    # We will drive the robot at a maximum speed of 0.3

    if abs(angle_diff) > 0.1:
        velocity_msg.angular.z = 0.4 if angle_diff > 0 else -0.4
        velocity_msg.linear.x = 0
    else:
    	velocity_msg.linear.x = 0.2

    # update the new rotation for the next loop
    last_rotation = rotation
    
    return velocity_msg


if __name__ == "__main__":


    # Subdivision of angles in the lidar scanner
    angle_threshold = 60
    # Obstacle threshhold, objects below this distance are considered obstacles
    obs_threshold = 0.3
    position_rob = None
    yaw_rob = None
    #this is a global variable that keeps handles the orders for the robot to follow
    #if there's a detected object, "flag" is turned to True
    #and the angular_vel and sleep values are calculated appropriately
    robot_order = {"flag": False, "angular_velocity": 0.0, "sleep_time": 0}
    
    #This dict keeps track of the distance measures for each region
    div_distance = { "0":[], "1":[], "2":[], "3":[], "4":[], "5":[], "6":[], "7":[] }
    
    #This dict keeps track of the cost of each region to move to 0th location
    div_cost = { "0":0, "1":1, "2":2, "3":3, "4":4, "5":-3, "6":-2, "7":-1 } 

    # Initialize your ROS node
    rospy.init_node("my_bot_controller")
    #Subscribe to the "/scan" topic in order to read laser scans data from it
    rospy.Subscriber("/scan", LaserScan, Callback)
    #rospy.Subscriber('/odom', Odometry, get_odom_data)
    # Set up a publisher to the /cmd_vel topic
    pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
    # Declare a message of type Twist
    velocity_msg = Twist()
    # publish the velocity at 10 Hz (10 times per second)
    rate = rospy.Rate(10)
    
    tf_listener = tf.TransformListener()
    # parent frame for the listener
    parent_frame = '/odom'
    # child frame for the listener
    child_frame = '/base_link'
    # gains for the proportional controllers. These values can be tuned.
    k_h_gain = 0.7  #linear
    k_v_gain = 1.0  #angular

    try:
        tf_listener.waitForTransform(parent_frame, child_frame, rospy.Time(), rospy.Duration(1.0))
    except (tf.Exception, tf.ConnectivityException, tf.LookupException):
        rospy.loginfo("Cannot find transform between {p} and {c}".format(p=parent_frame, c=child_frame))
        rospy.signal_shutdown("tf Exception")



    # Taking user input for the goal state, if user enters values lower than 0 or higher than 5, the program ends prompting the user with invalid arguments   
    if len(sys.argv) == 2:
        try:
            goal_state = int(sys.argv[1])
            if goal_state < 0 or goal_state > 5:
                sys.exit('Invalid arguments passed')
                
            print(goal_state)		
            # get the goal to reach from arguments passed to the command line
            global state_list
            state_list = [
            (1.00, 6.2),  # Punto 1
            (2.00, 5.50),  # Punto 1
            (3.50, 4.50),  # Punto 3
            (6, 4.50),  # Punto 3
            (6, 5.5),  # Punto 3
            (6.75, 5.50),  # Punto 4
            (6.75, 6.50),  # Punto 4
            (7.5, 6.50),  # Punto 4
            (7.5, 1.50),  # Punto 4
            (6.75, 1.50),  # Punto 5
            (5.00, 1.50),  # Punto 6
            (5.00, 2.50),  # Punto 7
            (4.2, 2.50),  # Punto 7
            (4.2, 3.50),  # Punto 7
            (2.50, 3.55),  # Punto 8
            (4.2, 3.50),  # Punto 7
            (4.2, 0.5),  # Punto 7
            (2.50, 0.5),  # Punto 9
            (0.5, 0.5),  # Punto 9
            (0.50, 4.50)   # Punto 10
        ]	
            goal = state_list[goal_state]
            goal_x, goal_y = goal[0], goal[1]
            
            # get current pose of the robot from the /odom topic
            (position, rotation) = get_odom_data()

            # compute the distance from the current position to the goal
            distance_to_goal = compute_distance(position.x, position.y, goal_x, goal_y)
            
            tick = time.time()

            while not rospy.is_shutdown():
                
                while distance_to_goal > 0.3:   # Goal threshold 0.07 

                    # When robot is stuck in a loop this condition comes into play___________________________
                    # This will give a backward jerk to robot if it doesn't reach the goal in 60 seconds
                    tock = time.time()
                    time_diff = tock - tick
                    if time_diff > 60:
                        for i in range(10):
                            rospy.loginfo("Giving backward jerk")
                            velocity_msg.linear.x = -0.5
                            velocity_msg.linear.y = 0
                            velocity_msg.angular.z = -10
                            pub.publish(velocity_msg)
                            rate.sleep()
                        tick = time.time()
                    #________________________________________________________________________________________

                    # get current pose of the robot from the /odom topic
                    (position, rotation) = get_odom_data()
            
                    # compute the distance from the current position to the goal
                    distance_to_goal = compute_distance(position.x, position.y, goal_x, goal_y)
                    rospy.loginfo("distance to goal {0}". format(distance_to_goal))

                    if distance_to_goal < 0.25:
                        vel = go_to_goal(goal_x, goal_y, position, rotation, distance_to_goal)
                        pub.publish(vel)
                    else:
                        vel = go_to_goal(goal_x, goal_y, position, rotation, distance_to_goal)
                        pub.publish(vel)
       
                
                else:
                    if goal_state + 1 < len(state_list):
                    	goal_state += 1
                    	goal = state_list[goal_state]
                    	goal_x, goal_y = goal[0], goal[1]
                    	distance_to_goal = compute_distance(position.x, position.y, goal_x, goal_y)
                    	tick = time.time()
                    	
                    else:
                    	rospy.loginfo("Goal Reached")
                    	velocity_msg.linear.x = 0.0
                    	velocity_msg.angular.z = 0.0
                    	pub.publish(velocity_msg)
                    	rate.sleep()
                    	break

        except Exception as e:
            print(e)
            sys.exit('Incorrect arguments passed')
    else:
        sys.exit('Not enough arguments passed to the command line')
