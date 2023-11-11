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
        timer_period = 0.1  # seconds
        # Initialize a timer that excutes call back function every 0.5 seconds
        self.timer = self.create_timer(timer_period, self.move2goal)
        self.odometry = Odometry()
        self.position = self.odometry.pose.pose.position
        self.orientation = self.odometry.pose.pose.orientation
        self.distance_tolerance = 0.01
        self.goal_pose = Pose()
        # Get the input from the user.
        self.get_user_input()

    def get_user_input(self):
        time.sleep(0.2)
        self.goal_pose.x = float(input("Set your x goal: "))
        self.goal_pose.y = float(input("Set your y goal: "))
        
    def update_pose(self, data):
        self.odometry = data
        self.position = self.odometry.pose.pose.position
        self.orientation = self.odometry.pose.pose.orientation
        self.position.x = round(self.position.x, 4)
        self.position.y = round(self.position.y, 4)
        
    def quater2eulerZ(self):
    	q_w = self.orientation.w
    	q_x = self.orientation.x
    	q_y = self.orientation.y
    	q_z = self.orientation.z
    	
    	t1 = 2.0 * (q_w * q_z + q_x * q_y)
    	t2 = 1.0 - 2.0 * (q_y * q_y + q_z * q_z)
    	return atan2(t1,t2)

    def euclidean_distance(self):
        return sqrt(pow((self.goal_pose.x - self.position.x), 2) + pow((self.goal_pose.y - self.position.y), 2))

    def linear_vel(self, constant=1.5):
    	lin_vel = float(constant * self.euclidean_distance())
    	if (lin_vel > 0.4):
    	    lin_vel = 0.4
    	if (lin_vel < -0.4):
    	    lin_vel = -0.4
    	return lin_vel

    def steering_angle(self):
        return atan2(self.goal_pose.y - self.position.y, self.goal_pose.x - self.position.x)

    def angular_vel(self, constant=1.5):
    	ang_vel = float(constant * (self.steering_angle() - self.quater2eulerZ()))
    	if (self.steering_angle() > 1.57 and self.quater2eulerZ() < -1.57) or (self.steering_angle() < -1.57 and self.quater2eulerZ() > 1.57):
    	    ang_vel = -1 * (ang_vel-3.14)
    	if  (ang_vel > 1.0):
    	    ang_vel = 1.0
    	if  (ang_vel < -1.0):
            ang_vel = -1.0
    	return ang_vel
    	

    def move2goal(self):
        """Moves the turtle to the goal."""

        vel_msg = Twist()

        if ((self.euclidean_distance() >= self.distance_tolerance)):
            # Porportional controller.
            # https://en.wikipedia.org/wiki/Proportional_control

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
            self.get_logger().info('x: ' + str(self.position.x) + ' y: ' + str(self.position.y))
        else:
	    #Stopping our robot after the movement is over.
            vel_msg.linear.x = 0.0
            vel_msg.angular.z = 0.0
            self.velocity_publisher_.publish(vel_msg)
            self.get_logger().info('Arrived goal')
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
