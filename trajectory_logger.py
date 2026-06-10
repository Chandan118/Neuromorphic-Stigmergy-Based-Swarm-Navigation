import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import csv
import time

class TrajectoryLogger(Node):
    def __init__(self):
        super().__init__('trajectory_logger')
        self.num_robots = 50
        self.subscribers_list = []
        
        # Open CSV file for logging
        self.csv_file = open('experiment_results.csv', mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['timestamp', 'robot_id', 'x', 'y', 'z'])
        
        self.get_logger().info('Initializing 50 Odometry subscribers...')
        for i in range(self.num_robots):
            topic_name = f'/robot_{i}/odom'
            sub = self.create_subscription(
                Odometry,
                topic_name,
                lambda msg, robot_id=i: self.odom_callback(msg, robot_id),
                10
            )
            self.subscribers_list.append(sub)
            
        self.get_logger().info('Trajectory Logger successfully started. Recording data to experiment_results.csv')

    def odom_callback(self, msg, robot_id):
        # Extract timestamp and position
        current_time = time.time()
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
        
        # Write to CSV
        self.csv_writer.writerow([current_time, robot_id, x, y, z])

    def destroy_node(self):
        self.csv_file.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    logger_node = TrajectoryLogger()
    
    try:
        rclpy.spin(logger_node)
    except KeyboardInterrupt:
        logger_node.get_logger().info('Shutting down logger. Results saved.')
    finally:
        logger_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
