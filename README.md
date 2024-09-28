**Problem:**
Implement an Open loop controller node to control the Turtlebot3
The open loop controller node should decide a reasonable distance and time of travel (for eg. 1m in 10 seconds) and then send appropriate control commands to turtlebot 3 to reach the goal along the same line and stop. Note that this is meant to be a 1DOF problem. So the start point and the goal point are along the same line and there is no need for steering. You can choose any velocity profile. The robot can move with a constant velocity the entire time, or you can design acceleration, steady state motion, and de-acceleration phases for the robot. The pose of the robot should be plotted over time. 
Consider these two scenarios:
Scenario 1: The TurtleBot moved at a constant velocity to reach a specific coordinate along a straight line, following the equation x=v⋅t, where x represents the displacement, v is the constant velocity, and t is the time.
Scenario 2: The TurtleBot begins moving with an initial acceleration (a), accelerating until it reaches a certain speed (v). It then continues to move at this constant velocity (v) over a distance of x2​. Finally, the TurtleBot decelerates with a negative acceleration (- a) until it comes to a complete stop.


This is a project to specify motion to the turtle bt in Gazebo environment using ROS2.
2 Scenarios are there and 2 separate nodes are made in the package tb_control

**1st scenario:**
Check the node file named tb_openLoop_scenario1.py in tb_control package

**2ndt scenario:**
Check the node file named tb_openLoop_scenario2.py in tb_control package

Note: use **ros2 run  tb_control  tb_openLoop_scenario1** to launch 1st scenario case
      use **ros2 run  tb_control  tb_openLoop_scenario2** to launch 2nd scenario case
