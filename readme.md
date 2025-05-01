cd ~/final_ws/

ros2 launch final_project final_project.launch.py

#Change the map location to the given map in final_project package
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=~∕src/final_project/maps/final2_map.yaml


-> The package given has 2 .cpp and 2 .hpp files. We wrote the navigation code separately and called it in the node.cpp file. The node is called TheNode and the executable is called theNode.

-> First source your terminal and run ros2 launch final_project final_project.launch.py to initialize the world. Then open a new terminal and run the command ros2 run group4_final theNode to make the turtlebot move to the camera positions.


-> We sometimes had the turtlebot-3 getting stuck in RVIZ but it moves on gazebo. In such a condition restart the terminal and run it again.

-> This is the link of our simulation just incase the above problems occur multiple times when you run our code: https://drive.google.com/file/d/1PWpEiC-xKU92htG33iT_v03Zv6sRSVHw/view?usp=sharing


