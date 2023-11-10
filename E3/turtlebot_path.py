import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from nav_msgs.msg import Odometry
from math import pow, atan2, sqrt
import time

class TurtleBot(Node):

    def __init__(self):
    	# Initialize the node
        super().__init__('turtlebot_controller')
        # Initialize the publisher
        self.velocity_publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        # Initialize the subscriber
        self.subscriber_ = self.create_subscription(Odometry, '/odom', self.update_pose, 10)
        # Initialize a timer that excutes call back function every 0.1 seconds
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.move2goal)
        self.odometry = Odometry()
        self.position = self.odometry.pose.pose.position
        self.orientation = self.odometry.pose.pose.orientation
        self.goal_pose = Pose()
        # Set tolerance
        self.distance_tolerance = 0.01
        self.angle_tolerance = 0.01
        # Define path
        self.goals = [[0.5, -0.5, 1.57], [0.5, 1.5, 0.0], [1.5, 1.5, -1.57], [1.5, 0.5, -3.14], [-2.0, 0.5, -1.57], [-2.0, -0.5, 0.0]]
        self.index = 0
        self.flag = False
        self.get_goal()

    # Get target
    def get_goal(self):
        time.sleep(0.2)
        self.goal_pose.x = self.goals[self.index][0]
        self.goal_pose.y = self.goals[self.index][1]
        self.goal_pose.theta = self.goals[self.index][2]
        self.get_logger().info('Current goal: x: ' + str(self.goal_pose.x) + ' y: ' + str(self.goal_pose.y))
    
    # Update current position
    def update_pose(self, data):
        self.odometry = data
        self.position = self.odometry.pose.pose.position
        self.orientation = self.odometry.pose.pose.orientation
        self.position.x = round(self.position.x, 4)
        self.position.y = round(self.position.y, 4)

    # Converse feedback angle from Quaternion to Euler angle
    def feedback_angle(self):
    	q_w = self.orientation.w
    	q_x = self.orientation.x
    	q_y = self.orientation.y
    	q_z = self.orientation.z
    	
    	t1 = 2.0 * (q_w * q_z + q_x * q_y)
    	t2 = 1.0 - 2.0 * (q_y * q_y + q_z * q_z)
    	return atan2(t1,t2)

    # Get Euclidean distance between target and current position
    def euclidean_distance(self):
        return sqrt(pow((self.goal_pose.x - self.position.x), 2) + pow((self.goal_pose.y - self.position.y), 2))

    # Apply propositional control to linear velocity and set maximum speed
    def linear_vel(self, constant=1.5):
    	lin_vel = float(constant * self.euclidean_distance())
    	if (lin_vel > 0.4):
    	    lin_vel = 0.4
    	if (lin_vel < -0.4):
    	    lin_vel = -0.4
    	return lin_vel

    # Get required steering angle to target
    def steering_angle(self):
        return atan2(self.goal_pose.y - self.position.y, self.goal_pose.x - self.position.x)

    # Apply propositional control to angular velocity and set maximum speed
    def angular_vel(self, constant=1.5):
        ang_vel = float(constant * (self.steering_angle() - self.feedback_angle()))
        if (abs(self.steering_angle() - self.feedback_angle()) > 3.14):
            ang_vel *= -1
        if  (ang_vel > 1.0):
    	    ang_vel = 1.0
        if  (ang_vel < -1.0):
            ang_vel = -1.0
        return ang_vel

    # Get angular difference between target and current angle
    def angular_difference(self):
    	return (self.goal_pose.theta - self.feedback_angle())

    # Get turning velocity
    def turning_vel(self, constant=2):
        t_vel = float(constant * self.angular_difference())
        # If difference is larger than 180 degree, turn the other direction to achieve shortest rotation
        if (abs(self.angular_difference()) > 3.14):
            t_vel *= -1
        if t_vel > 0.8:
            t_vel = 0.8
        elif t_vel < -0.8:
            t_vel = -0.8
        return t_vel

    # Move to target
    def move2goal(self):
        """Moves the turtle to the goal."""
        vel_msg = Twist()

        # Move turtlebot until it is within set distance tolerance
        if ((self.euclidean_distance() >= self.distance_tolerance) and self.flag == False):
            # Linear velocity in the x-axis.
            vel_msg.linear.x = self.linear_vel()
            vel_msg.linear.y = 0.0
            vel_msg.linear.z = 0.0
            # Angular velocity in the z-axis.
            vel_msg.angular.x = 0.0
            vel_msg.angular.y = 0.0
            vel_msg.angular.z = self.angular_vel()
            # Publishing our vel_msg
            self.velocity_publisher_.publish(vel_msg)
        else:
            # Rotate turtlebot if it is not within set angle tolerance
            self.flag = True
            if (abs(self.angular_difference()) >= self.angle_tolerance):
                vel_msg.linear.x = 0.0
                # Angular velocity in the z-axis.
                vel_msg.angular.z = self.turning_vel()
                self.velocity_publisher_.publish(vel_msg)
            else:
                # If within angle tolerance, move to next target
                if (self.index < len(self.goals)-1):
                    vel_msg.linear.x = 0.0
                    vel_msg.angular.z = 0.0
                    self.velocity_publisher_.publish(vel_msg)
                    #self.get_logger().info('Turtlebot arrived x: ' + str(self.position.x) + ' y: ' + str(self.position.y))
                    self.index += 1
                    self.get_goal()
                    self.flag = False
                    time.sleep(0.1)
                else:
        	    # If there is no next target, stop the robot.
                    vel_msg.linear.x = 0.0
                    vel_msg.angular.z = 0.0
                    self.velocity_publisher_.publish(vel_msg)
                    self.get_logger().info('Path completed')
                    self.destroy_node()
                    #rclpy.shutdown()
                    quit()

def main(args=None):
    rclpy.init(args=args)
    
    # create an object for GoForward class
    node = TurtleBot()
    # continue untill interrupted
    rclpy.spin(node)
    # clear the node
    node.destroy_node()
    rclpy.shutdown()
    	
if __name__ == '__main__':
    main()
