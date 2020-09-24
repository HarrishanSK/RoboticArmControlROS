Student Name: Harrishan Sureshkumar
Student ID: 150294245

README FILE: Coursework 2 ECS7004P (2020 March)

To run the package:

1) Download and install the following packages:

	- git clone -b melodic-devel https://github.com/ros-planning/moveit_tutorials.git
	- git clone -b melodic-devel https://github.com/ros-planning/panda_moveit_config.git

where 'melodic' is the name of your ROS distribution.

2) Unzip the AR_week8_test.zip folder in the src folder of your catkin workspace.

3) Build the catkin workspace by running the command 'catkin_make'.

4) Run the 'roscore' command in 1 terminal and then run the following commands in four different terminals:

	- roslaunch panda_moveit_config demo.launch
	- rosrun AR_week8_test square_size_generator.py
	- rosrun AR_week8_test move_panda_square.py
	- rosrun rqt_plot rqt_plot

NOTE: Please make sure your computer or virtual machine has a suitable amount of RAM and CPU allocated to run the code to be used with ROS, R-viz and MoveIt.
