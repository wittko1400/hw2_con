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


LINEAR_VEL = 0.22
STOP_DISTANCE = 0.15
LIDAR_ERROR = 0.05
LIDAR_AVOID_DISTANCE = 0.8
SAFE_STOP_DISTANCE = STOP_DISTANCE + LIDAR_ERROR
RIGHT_SIDE_INDEX = 270
RIGHT_FRONT_INDEX = 210
LEFT_FRONT_INDEX= 150
LEFT_SIDE_INDEX= 90
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
        timer_period = 0.5
        self.pose_saved=''
        self.cmd = Twist()
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.last_turn_time_ns = self.get_clock().now().nanoseconds
        self.last_turn_time_secs = self.last_turn_time_ns / 1e9
        self.random_turn_time = 0.0
        self.stall_start_time = None
        self.stall_timer = None
        self.front_lidar_constant_time = None


        # Create a new text file with a unique name based on the current timestamp
        #timestamp = datetime.datetime.now().strftime("%Y%m%d%H%M%S")
        #file_name = f"robot_log_{timestamp}.txt"
        
        # Open the file in write mode and store the file object as an instance variable
        self.log_file = open(file_name, "w")
        self.log_file.write("Robot Log")
        self.log_file.write(f"Timestamp: {timestamp}\n\n")
        self.log_file.write("Time (s),X Position (m),Y Position (m)\n")
        
        
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
        (qx, qy, qz, qw) = (orientation.x, orientation.y, orientation.z, orientation.w)
        self.log_file.write(f"{posx}, {posy}\n")
        
        #self.get_logger().info('self position: {},{},{}'.format(posx,posy,posz));
        # Create or open the CSV file in write mode
        #with open(csv_file_path, mode='w', newline='') as csv_file:
            #position = msg2.pose.pose.position
            # Create a CSV writer object
            #csv_writer = csv.writer(csv_file)

            # Assuming you have posx, posy, and posz variables
            #position_data = [position.x, position.y, position.z]

            # Write the position data to the CSV file
            #csv_writer.writerow(position_data)

            # Optionally, you can log the position to the console as well
        self.get_logger().info('self position: {}, {}, {}'.format(posx, posy, posz))
        # similarly for twist message if you need
        self.pose_saved=position
        return None
    
        
    def timer_callback(self):
        if (len(self.scan_cleaned)==0):
            self.turtlebot_moving = False
            return
            
        #left_lidar_samples = self.scan_cleaned[LEFT_SIDE_INDEX:LEFT_FRONT_INDEX]
        #right_lidar_samples = self.scan_cleaned[RIGHT_FRONT_INDEX:RIGHT_SIDE_INDEX]
        #front_lidar_samples = self.scan_cleaned[LEFT_FRONT_INDEX:RIGHT_FRONT_INDEX]
        
        left_lidar_min = min(self.scan_cleaned[LEFT_SIDE_INDEX:LEFT_FRONT_INDEX])
        right_lidar_min = min(self.scan_cleaned[RIGHT_FRONT_INDEX:RIGHT_SIDE_INDEX])
        front_lidar_min = min(self.scan_cleaned[LEFT_FRONT_INDEX:RIGHT_FRONT_INDEX])

        #self.get_logger().info('left scan slice: "%s"'%  min(left_lidar_samples))
        #self.get_logger().info('front scan slice: "%s"'%  min(front_lidar_samples))
        #self.get_logger().info('right scan slice: "%s"'%  min(right_lidar_samples))

        current_time_ns = self.get_clock().now().nanoseconds                        # get the current time in nanoseconds
        current_time_secs = current_time_ns / 1e9                                   # convert the current time to seconds
        time_since_turn = current_time_secs - self.last_turn_time_secs              # calculate the time since the last turn
        self.random_turn_time = random.randint(2, 5)                                # random turn time between 2 and 5 seconds

        if front_lidar_min < SAFE_STOP_DISTANCE:
            if self.turtlebot_moving == True:
                # Halt the robot if it is too close to an obstacle
                self.cmd.linear.x = 0.0 
                self.cmd.angular.z = 0.0
                self.publisher_.publish(self.cmd)
                self.turtlebot_moving = False
                self.get_logger().info('Stopping')
            else:
                self.get_out_of_stall()     # Get the robot out of the stall/stop
        elif front_lidar_min < LIDAR_AVOID_DISTANCE:
                # Turn away from the obstacle
                self.cmd.linear.x = 0.07 
                if (right_lidar_min > left_lidar_min):
                   self.cmd.angular.z = -0.4
                else:
                   self.cmd.angular.z = 0.4
                self.publisher_.publish(self.cmd)
                self.get_logger().info('Turning')
                self.turtlebot_moving = True
        else:
            # Explore the environment if there is no obstacle
            self.cmd.linear.x = 0.3

            # Make a random turn within a time interval between 2-5 seconds
            if time_since_turn > self.random_turn_time:
                self.cmd.angular.z = random.uniform(-0.9, 0.9)          # random angular velocity between -3 and 3
                self.last_turn_time_secs = current_time_secs        # reset the timer
            else:
                self.cmd.angular.z = 0.0
            
            self.publisher_.publish(self.cmd)
            self.get_logger().info('Exploring')
            self.turtlebot_moving = True

        self.get_logger().info('Distance of the obstacle : %f' % front_lidar_min)
        self.get_logger().info('I receive: "%s"' %
                               str(self.odom_data))
        if self.stall == True:
            self.get_logger().info('Stall reported')
            self.get_out_of_stall()     # Get the robot out of the stall
        
        # Display the message on the console
        self.get_logger().info('Publishing: "%s"' % self.cmd)

    def get_out_of_stall(self):
        # If the robot is stopped due to front_lidar_min < SAFE_STOP_DISTANCE, then move out of the stop
        if self.turtlebot_moving == False:
            self.cmd.linear.x = -0.4
            self.cmd.angular.z = random.uniform(-0.6, 0.6)
            self.publisher_.publish(self.cmd)
            self.get_logger().info('Backing out of stall')
            self.turtlebot_moving = True


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
