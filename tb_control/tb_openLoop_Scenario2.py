
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import time
import matplotlib.pyplot as plt

class OpenLoopController(Node):
    def __init__(self):
        super().__init__('tb_openLoop_Scenario2')
        self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 20)
        self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.odom_callback, 20)
        self.timer = self.create_timer(0.1, self.Robot)
        self.start_time = time.time()
       
       # Robot motion plan: Accelerate till the velocity reaches 'max_velocity'. Then will cover 'distance_const_Vel' 
       # with speed max_veloctity. Then finaaly will start decelrating with 'decelration' till stop
       
       # Parameters
        self.acceleration = 0.2  # m/s^2
        self.max_velocity = 1.0   # m/s
        self.distance_const_Vel = 3.0  # In meters
        self.deceleration = -0.2  # m/s^2
        
        # Calculate time for acceleration
        self.time_acceleration = self.max_velocity / self.acceleration
        self.distance_acceleration = 0.5 * self.acceleration * (self.time_acceleration ** 2)
        
        # Calculate time for constant velocity
        self.time_to_const_Vel = self.distance_const_Vel/self.max_velocity  #Since Maximum velocity is equal to constant velocity of travel

        # Calculate time for deceleration
        self.time_deceleration = -1*self.max_velocity / self.deceleration
        self.distance_acceleration = (self.max_velocity*self.time_deceleration) + (0.5 * self.deceleration * (self.time_deceleration ** 2))
        
        # Total time of travel
        self.total_time = self.time_acceleration + self.time_to_const_Vel + self.time_deceleration
        
        self.moving = True
        self.state = "accelerating"
        
        #Let's initialize variable
        self.time_data = []
        self.x_data = []

    def odom_callback(self, msg):
        # Extract the robot's pose from Odometry
        x = msg.pose.pose.position.x
        self.time_data.append(time.time() - self.start_time)
        self.x_data.append(x)

    def Robot(self):    # Function for Robot motion
        elapsed_time = time.time() - self.start_time
        
        # Debugging
        self.get_logger().info(f"Elapsed Time: {elapsed_time:.2f}, Acceleration Time: {self.time_acceleration:.2f}, Moving: {self.moving}")

        
        if elapsed_time < self.time_acceleration and self.moving:
            # Acceleration
            vel_msg = Twist()
            vel_msg.linear.x = self.acceleration * elapsed_time
            self.velocity_publisher.publish(vel_msg)
        
        elif elapsed_time < (self.time_acceleration + self.time_to_const_Vel) and self.moving:
            # Constant velocity
            vel_msg = Twist()
            vel_msg.linear.x = self.max_velocity
            self.velocity_publisher.publish(vel_msg)
        
        elif elapsed_time < self.total_time and self.moving:
            # Decelerate
            vel_msg = Twist()
            vel_msg.linear.x = self.max_velocity + (self.deceleration * (elapsed_time - (self.time_acceleration + self.time_to_const_Vel)))
            self.velocity_publisher.publish(vel_msg)
        
        else:
            # Stop the robot
            vel_msg = Twist()
            self.velocity_publisher.publish(vel_msg)
            if self.moving:
                self.get_logger().info('Reached the goal, stopping the robot.')
            self.moving = False
            self.plot_data()

     
    def plot_data(self):
        plt.figure()
        plt.plot(self.time_data, self.x_data, marker='o')
        plt.title('Robot Position over Time')
        plt.xlabel('Time Elapsed (s)')
        plt.ylabel('X Position (m)')
        plt.grid()
        plt.show()
            
def main(args=None):
    rclpy.init(args=args)
    tb_openLoop_Scenario2 = OpenLoopController()
    rclpy.spin(tb_openLoop_Scenario2)
    tb_openLoop_Scenario2.destroy_node()
    rclpy.shutdown()

# def main():
    # print('Hi from tb_control.')


if __name__ == '__main__':
    main()
