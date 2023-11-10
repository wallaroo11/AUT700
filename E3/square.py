import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("sqaure")
    publisher = node.create_publisher(Twist, 'turtle1/cmd_vel', 10)

    for _ in range(4):
        msg = Twist()
        msg.linear.x = 2.0
        publisher.publish(msg)
        time.sleep(2)
        
        msg = Twist()
        msg.angular.z = 1.57
        publisher.publish(msg)
        time.sleep(2)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
