
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import time
import matplotlib.pyplot as plt

class OpenLoopController(Node):
    def __init__(self):
        super().__init__('tb_openLoop_Scenario1')
        self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 20)
        self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.odom_callback, 20)
        self.timer = self.create_timer(0.1, self.Robot)
        self.start_time = time.time()
        
        # Robot's motion: Will move at a constant velocity of 0.2m/s for 20 seconds. Distance can be calculated accordingly 
        
        # Parameters
        # self.duration = 20  # In seconds
        self.velocity = 0.2  # In m/s
        self.distance = 4.0  # In meters
        self.movement_time = self.distance / self.velocity
        self.moving = True
        
        
        self.time_data = []
        self.x_data = []

    def odom_callback(self, msg):
        # Extract the robot's pose from Odometry
        x = msg.pose.pose.position.x
        self.time_data.append(time.time() - self.start_time)
        self.x_data.append(x)


    def Robot(self):    # Function for robot motion  
        elapsed_time = time.time() - self.start_time
        
        # Debugging
        self.get_logger().info(f'Elapsed Time: {elapsed_time:.2f}, Current Position: {self.x_data[-1] if self.x_data else 0:.2f}')
        
        
        if elapsed_time < self.movement_time and self.moving:
            vel_msg = Twist()
            vel_msg.linear.x = self.velocity  # Constant velocity
            self.velocity_publisher.publish(vel_msg)
        else:
            vel_msg = Twist()
            self.velocity_publisher.publish(vel_msg)  # Stop the robot
            if self.moving:
                self.get_logger().info('Reached the goal, stopping the robot.')
                self.plot_data()
            self.moving = False
     
    def plot_data(self):
        plt.figure()
        plt.plot(self.time_data, self.x_data, marker='o')
        plt.title('Robot Position over Time')
        plt.xlabel('Time Elapsed (s)')
        plt.ylabel('X Position (m)')
        plt.grid() 
        plt.axis('equal') 
        plt.show()
            
def main(args=None):
    rclpy.init(args=args)
    tb_openLoop_Scenario1 = OpenLoopController()
    rclpy.spin(tb_openLoop_Scenario1)
    tb_openLoop_Scenario1.destroy_node()
    rclpy.shutdown()

# def main():
    # print('Hi from tb_control.')


if __name__ == '__main__':
    main()
