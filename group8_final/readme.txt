809y - Group_8 Final Project

Project Description:- 

THe project consists of a Turtlebot which uses FollowWaypoint Action Client. The bot detects the parts using logical cameras, and navigate using the order described by the parameters. 


Steps to run the code:-

1. Colcon build

2. Source the package

3. Launch Project - ros2 launch final_project final_project.launch.py

4. Run the Broadcaster file - ros2 run group8_final broadcaster 
	
5. Launch WaffleBot - ros2 launch group8_final waffle.launch.py
	-The launch file will load the parameters and launch the navigation file with it.
	-The system will take a few seconds to initialize the turtlebot.
	-"SOMETIMES IT MAY TAKE 2 TO 3 TRIES ON INITIALIZING. PLEASE RUN THE FILE AGAIN".Check the 			RVIZ file to see if the turtlebot is initialized.
	-The loop for goal execution takes some time, please wait for it to load.
	-The terminal will show "Loop Executed" 5 times.
	-The terminal will show "Waypoint accepted by server, waiting for result" and the turtle 			bot will start moving to the goals


