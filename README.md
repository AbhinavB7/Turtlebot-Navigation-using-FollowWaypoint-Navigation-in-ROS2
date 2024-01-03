# Turtlebot-Navigation-using-FollowWaypoint-Navigation-in-ROS2

The Turtlebot will read the ArUCo Marker and retrieve the parameters from the params.yaml file, the robot will navigate through the position of each part in the order given in the yaml file.

![Screenshot from 2023-12-28 19-14-55](https://github.com/AbhinavB7/Turtlebot-Navigation-using-FollowWaypoint-Navigation-in-ROS2/assets/87815926/da69551e-b7bc-4249-aca7-b4de9c57b32e)
The above image is the Environment of the Project

Navigation of Turtlebot when the Aruco Marker Id is 0.
https://github.com/AbhinavB7/Turtlebot-Navigation-using-FollowWaypoint-Navigation-in-ROS2/assets/87815926/95a83597-ff13-4a8e-b1bd-ec1e342f33d2

Navigation of Turtlebot when the Aruco Marker Id is 1.
https://github.com/AbhinavB7/Turtlebot-Navigation-using-FollowWaypoint-Navigation-in-ROS2/assets/87815926/2260130e-f062-4f40-b8ca-5e2ffbb02857


We create a ROS Package where we:
- Capture the positions of the part frames from the logical cameras and transform w.r.t to the map.
- Store the positions of the parts.
- Read the ArUco marker's id and obtain the waypoint turtlebot needs to follow from the params.yaml file.
- Find the parts w.r.t their part color number and part type number and retreive their positions accordingly.
- The bot will initialize in the RVIZ environment and the waypoints generate in the order.
- The turtle bot will navigate around the map sequentially.

![image](https://github.com/AbhinavB7/Turtlebot-Navigation-using-FollowWaypoint-Navigation-in-ROS2/assets/87815926/65c26274-c2f0-49e1-bb1b-129385f6951b)

RVIZ after initialization of the robot.


