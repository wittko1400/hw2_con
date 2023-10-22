import rclpy
# import the ROS2 python libraries
from rclpy.node import Node
# import the Twist module from geometry_msgs interface
from geometry_msgs.msg import Twist
# import the LaserScan module from sensor_msgs interface
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
# import Quality of Service library, to set the correct profile and reliability in order to read sensor data.
from rclpy.qos import ReliabilityPolicy, QoSProfile
import math, random, time, os, datetime, csv


# Generate a unique timestamp for the CSV file name
timestamp = datetime.datetime.now().strftime("%Y%m%d%H%M%S")
file_name = f'position_data_{timestamp}.txt'  # Example: position_data_20230927153045.csv


class RandomWalk(Node):
    def __init__(self):
        # Initialize the publisher
        super().__init__('random_walk_node')
        self.scan_cleaned = []
        self.stall = False
        self.turtlebot_moving = False
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscriber1 = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback1,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.subscriber2 = self.create_subscription(
            Odometry,
            '/odom',
            self.listener_callback2,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.laser_forward = 0
        self.odom_data = 0

        #initialize the x,y,z positions to be able to access in timer_callback
        self.odom_x = 0.0  # Initialize the x position
        self.odom_y = 0.0  # Initialize the y position
        self.odom_z = 0.0  # Initialize the z position

        self.odom_save_x = 0.0
        self.odom_save_y = 0.0
        self.odom_save_z = 0.0


        timer_period = 0.5
        self.pose_saved=''
        self.cmd = Twist()
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.meter = False
        self.fiveMeter = False
        self.tenDegrees = False
        self.oneEightyDegrees = False
        self.threeSixtyDegrees = False
        
        # Create a new text file with a unique name based on the current timestamp
        #timestamp = datetime.datetime.now().strftime("%Y%m%d%H%M%S")
        #file_name = f"robot_log_{timestamp}.txt"
       
        # Open the file in write mode and store the file object as an instance variable
        self.log_file = open(file_name, "w")
        self.log_file.write("Robot Log")
        self.log_file.write(f"Timestamp: {timestamp}\n\n")
        self.log_file.write("Time (s),X Position (m),Y Position (m), Rotate Z (m)\n")
       
       
    def listener_callback1(self, msg1):
        #self.get_logger().info('scan: "%s"' % msg1.ranges)
        scan = msg1.ranges
        self.scan_cleaned = []
       
        #self.get_logger().info('scan: "%s"' % scan)
        # Assume 360 range measurements
        for reading in scan:
            if reading == float('Inf'):
                self.scan_cleaned.append(3.5)
            elif math.isnan(reading):
                self.scan_cleaned.append(0.0)
            else:
                self.scan_cleaned.append(reading)


    def listener_callback2(self, msg2):
        position = msg2.pose.pose.position
        orientation = msg2.pose.pose.orientation
        (posx, posy, posz) = (position.x, position.y, position.z)

        #fill data to the variables used in timer_callback
        self.odom_x = posx
        self.odom_y = posy
        

        (qx, qy, qz, qw) = (orientation.x, orientation.y, orientation.z, orientation.w)
        self.log_file.write(f"{posx}, {posy}, {qz}\n")

        self.odom_z= qz

        # Extract yaw angle using the provided code


        # Optionally, you can log the position to the console as well
        self.get_logger().info('self position: {}, {}, {}'.format(posx, posy, qz))
        # similarly for twist message if you need
        self.pose_saved=position
        return None
   
       
    def timer_callback(self):
        if (len(self.scan_cleaned)==0):
            self.turtlebot_moving = False
            return
       
        # 1 METER CODE
           
        """
        if self.odom_x <= 0.70:
            self.cmd.linear.x = 0.2
            self.publisher_.publish(self.cmd)
            self.get_logger().info('Moving')

        if self.odom_x >= 0.71 and self.odom_x <= 0.94:
            self.cmd.linear.x = 0.1
            self.publisher_.publish(self.cmd)
            self.get_logger().info('Moving')

            # Check if you've moved approximately 1 meter
        if self.odom_x >= 0.95:
            # Stop the robot
            self.cmd.linear.x = 0.0
            self.cmd.angular.z = 0.0
            self.publisher_.publish(self.cmd)
            self.turtlebot_moving = False
            self.get_logger().info('Stopped after moving approximately 1 meter')
            self.meter = True
        """

        # 5 METER CODE

        """
        if self.odom_x <= 4.6:
            self.cmd.linear.x = 0.2
            self.publisher_.publish(self.cmd)
            self.get_logger().info('Moving')

        if self.odom_x >= 4.6 and self.odom_x <= 4.95:
            self.cmd.linear.x = 0.1
            self.publisher_.publish(self.cmd)
            self.get_logger().info('slowing to goal')

            # Check if you've moved approximately 5 meter
        if self.odom_x >= 4.95:
            # Stop the robot
            self.cmd.linear.x = 0.0
            self.cmd.angular.z = 0.0
            self.publisher_.publish(self.cmd)
            self.turtlebot_moving = False
            self.get_logger().info('Stopped after moving approximately 5 meter')
            self.meter = True

        """

        # 10 DEGREE CODE
        
        """
        # Define the desired angle in radians (10 degrees)
        desired_angle = 0.053
        slow_angle = 0.035

        if self.odom_z <= desired_angle:
            self.cmd.angular.z = 0.12  # Angular velocity to rotate
            self.publisher_.publish(self.cmd)
            self.get_logger().info('Rotating')

        if self.odom_z <= desired_angle and self.odom_z >= slow_angle:
            self.cmd.angular.z = 0.03  # Angular velocity to rotate
            self.publisher_.publish(self.cmd)
            self.get_logger().info('slowing to goal')

            # Check if you've rotated approximately 10 degrees
        if self.odom_z >= desired_angle:
            # Stop the robot
            self.cmd.angular.z = 0.0
            self.publisher_.publish(self.cmd)
            self.turtlebot_rotating = False
            self.get_logger().info('Stopped after rotating approximately 10 degrees')
            self.degrees = True
            """

        # 180 DEGREE CODE

        """
        # Define the desired angle in radians (180 degrees)
        # Calculate the desired angle (180 degrees in radians)
          # 180 degrees in radians

    # Calculate the absolute difference between the current and desired angles
        desired_angle = .9996
        slow_angle = 0.875

        if self.odom_z <= desired_angle:
            self.cmd.angular.z = 0.2  # Angular velocity to rotate
            self.publisher_.publish(self.cmd)
            self.get_logger().info('Rotating')

        if self.odom_z <= desired_angle and self.odom_z >= slow_angle:
            self.cmd.angular.z = 0.03  # Angular velocity to rotate
            self.publisher_.publish(self.cmd)
            self.get_logger().info('slowing to goal')

            # Check if you've rotated approximately 10 degrees
        if self.odom_z >= desired_angle:
            # Stop the robot
            self.cmd.angular.z = 0.0
            self.publisher_.publish(self.cmd)
            self.turtlebot_rotating = False
            self.get_logger().info('Stopped after rotating approximately 180 degrees')
            self.degrees = True
        """

        # 360 DEGREE CODE

        #"""
        # Define the desired angle in radians (360 degrees)
        desired_angle = -0.10

        if self.odom_z >= 0:
            self.cmd.angular.z = 0.2  # Angular velocity to rotate
            self.publisher_.publish(self.cmd)
            self.get_logger().info('Rotating')


            # Check if you've rotated approximately 10 degrees
        elif self.odom_z <= desired_angle:
            self.cmd.angular.z = 0.2  # Angular velocity to rotate
            self.publisher_.publish(self.cmd)
            self.get_logger().info('Rotating')

        else:
            # Stop the robot
            self.cmd.angular.z = 0.0
            self.publisher_.publish(self.cmd)
            self.turtlebot_rotating = False
            self.get_logger().info('Stopped after rotating approximately 360 degrees')
            self.degrees = True

            
        #Go 0.3 m/s for 1m using position data

        #Go .08 m/s for 5m
        



def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)
    # declare the node constructor
    random_walk_node = RandomWalk()
    # pause the program execution, waits for a request to kill the node (ctrl+c)
    rclpy.spin(random_walk_node)
    # Explicity destroy the node
    random_walk_node.destroy_node()
    # shutdown the ROS communication
    rclpy.shutdown()

if __name__ == '__main__':
    main()
