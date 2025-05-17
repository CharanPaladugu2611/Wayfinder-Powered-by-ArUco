# 🧭 Wayfinder: ArUco-Driven Autonomous Navigation in ROS2
From detection to destination — a TurtleBot3 navigates autonomously using ArUco markers, logical cameras, and waypoint planning in a simulated warehouse.

---

## 📌 Overview
This ROS2 project implements an autonomous navigation system in a Gazebo simulation environment, using an ArUco marker to identify the task and trigger a dynamic navigation routine. The robot, a TurtleBot3 Waffle Pi, uses a combination of:

* 🏷️ ArUco marker detection
* 🧾 Dynamic YAML-based waypoint retrieval
* 🎥 Logical cameras for part localization
* 📍 TF-based transform projections
* 🛣️ Waypoint navigation via ```follow_waypoints``` action server

All components are orchestrated through a custom ROS2 node written in C++, enabling the robot to detect its mission marker, parse its target route from a config file, locate the parts, and navigate to them autonomously.

---

## 🎯 Objectives
* Detect an ArUco marker and determine which navigation task to execute
* Load a list of waypoints dynamically from a YAML file based on the ArUco ID
* Use static logical cameras to find the 3D poses of target parts
* Transform part poses into the global map frame
* Generate and send ordered goals to the follow_waypoints action server
* Complete the full mission autonomously in Gazebo simulation

---

## 🗺️ System Architecture
```
Gazebo Simulation Environment
├── ArUco Marker Detector
│   └── Publishes marker ID → triggers task
│
├── Custom Node (theNode)
│   ├── Subscribes to marker ID
│   ├── Loads waypoints from YAML
│   ├── Subscribes to logical camera topics (5 static cams)
│   ├── Transforms part poses to map frame (via TF2)
│   ├── Filters and formats goal poses
│   └── Sends goals to /follow_waypoints (Nav2)
│
└── Navigation Stack (Nav2)
    └── Executes path using local + global planners
```

---

## ⚙️ How It Works
1. Startup Simulation:
Launches Gazebo with a TurtleBot3 and static ArUco marker.

2. Marker Detection:
The robot subscribes to ```/aruco_markers``` to read the ID of the visible marker.

3. Waypoint Loading:
Using the marker ID, it dynamically loads a list of waypoints from a YAML config file.

4. Part Localization:
Subscribes to ```/logical_camera_X/image``` topics to find all part poses in camera frames. Transforms them to the map frame using TF.

5. Navigation:
Publishes the initial pose and sends the final sequence of transformed goals to the Nav2 stack using the ```/follow_waypoints``` action server.

---

## 🔧 Build & Run Instructions
### 🔨 Build
```
cd ~/your_ros2_ws
colcon build --packages-select group4_final
source install/setup.bash
```
### 🚀 Run
Start the simulation (from provided final_project package):

```ros2 launch final_project final_project.launch.py```

Then in a new terminal, run your node:

```ros2 run group4_final theNode```

---

## 📋 Dependencies
* ```ros2_aruco_interfaces```

* ```mage_msgs```

- ```nav2_msgs```

* ```tf2_ros, tf2_geometry_msgs```

* ```geometry_msgs, rclcpp, yaml_cpp_vendor```

All specified in ```CMakeLists.txt``` and ```package.xml```.

---

## 🔗 Video Demo
▶️ Watch it in action: [Demo Link]([/guides/content/editing-an-existing-page#modifying-front-matter](https://drive.google.com/file/d/1v56Dp-IDldkRQj9HL9xwYE209jGdJ08N/view?usp=drive_link))


