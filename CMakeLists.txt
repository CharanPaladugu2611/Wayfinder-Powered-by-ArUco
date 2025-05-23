cmake_minimum_required(VERSION 3.8)
project(group4_final)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(mage_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)

#Pratik
find_package(ros2_aruco_interfaces REQUIRED)

include_directories(include/group4_final)

add_executable(theNode src/theNode.cpp)
ament_target_dependencies(theNode 
    rclcpp 
    rclcpp_action
    rclcpp_components
    tf2
    tf2_geometry_msgs 
    tf2_ros 
    geometry_msgs 
    mage_msgs 
    nav2_msgs
    ros2_aruco_interfaces 
    yaml_cpp_vendor 
    ament_index_cpp

)
   



install(TARGETS theNode
    DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config  # Replace 'config' with the actual directory containing your YAML file
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
