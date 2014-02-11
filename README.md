Robo Surrogate
=============

This package enables fully immersive control of a robot using the Oculus Rift and Razer Hydra.

What it does:
- Render the Kinect point cloud and robot model to the Oculus Rift
- Use the Oculus head tracking to control the robot head
- Connect the Hydra to the joystick teleop (base motion, torso lift & gripper control)
- Track the Hydra motion with the robot gripper(s).

How to run:
- Connect your Hydra and Oculus Rift
- On the robot: `roslaunch robot_surrogate myrobot_robot.launch`
- On the desktop: `roslaunch robot_surrogate myrobot_desktop.launch`
