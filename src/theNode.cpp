#include "theNode.hpp"
/**
 * @brief Constructor for TheNode class.
 * 
 */
TheNode::TheNode() : Node("theNode"), cam1_received_(false), cam2_received_(false),
                     cam3_received_(false), cam4_received_(false), cam5_received_(false), aruco_received_(false) {

    auto qos = rclcpp::SensorDataQoS();

    subscription_aruco = create_subscription<ros2_aruco_interfaces::msg::ArucoMarkers>(
        "/aruco_markers", 10, std::bind(&TheNode::aruco_callback, this, std::placeholders::_1));

    subscription_cam1 = create_subscription<mage_msgs::msg::AdvancedLogicalCameraImage>(
        "/mage/camera1/image", qos, std::bind(&TheNode::cam1_callback, this, std::placeholders::_1));

    subscription_cam2 = create_subscription<mage_msgs::msg::AdvancedLogicalCameraImage>(
        "/mage/camera2/image", qos, std::bind(&TheNode::cam2_callback, this, std::placeholders::_1));

    subscription_cam3 = create_subscription<mage_msgs::msg::AdvancedLogicalCameraImage>(
        "/mage/camera3/image", qos, std::bind(&TheNode::cam3_callback, this, std::placeholders::_1));

    subscription_cam4 = create_subscription<mage_msgs::msg::AdvancedLogicalCameraImage>(
        "/mage/camera4/image", qos, std::bind(&TheNode::cam4_callback, this, std::placeholders::_1));

    subscription_cam5 = create_subscription<mage_msgs::msg::AdvancedLogicalCameraImage>(
        "/mage/camera5/image", qos, std::bind(&TheNode::cam5_callback, this, std::placeholders::_1));

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    client_ = rclcpp_action::create_client<nav2_msgs::action::FollowWaypoints>(this, "follow_waypoints");

    initial_pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "initialpose", 10);

    file_path = ament_index_cpp::get_package_share_directory("group4_final") + "/config/waypoint_params.yaml";
    config = YAML::LoadFile(file_path);
    marker_id = 100;  // Initialize marker_id to a value that is not 0 or 1
}// Implementation of TheNode constructor
/**
 * @brief callback function for processing Aruco marker data.
 * 
 * @param msg Aruco marker message.
 */
void TheNode::aruco_callback(const ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr msg) {
                marker_id = msg->marker_ids[0];
            
 
            if (marker_id == 0 or marker_id == 1) {
                    RCLCPP_INFO(this->get_logger(), "Stopping subscriber. Marker_id %d noted.", marker_id);
                    subscription_aruco.reset(); // Unsubscribe from the topic 
                }
        
        aruco_ = "aruco_" + std::to_string(marker_id);

        aruco_received_ = true;
        check_and_send_goal(); 

        // Accessing information for aruco_0 waypoints
        YAML::Node aruco = config["theNode"]["ros__parameters"][aruco_];

        // Loop to list all waypoints (wp1, wp2, wp3, wp4, wp5) under aruco_0
        for (const auto& wp : aruco) {
            std::string wp_name = wp.first.as<std::string>();  // Get the waypoint name (wp1, wp2, ...)
            std::string type = wp.second["type"].as<std::string>();  // Get the 'type' value
            std::string color = wp.second["color"].as<std::string>();  // Get the 'color' value

            RCLCPP_INFO(this->get_logger(), "Waypoint: %s, Type: %s, Color: %s",
                                wp_name.c_str(), type.c_str(), color.c_str());    
        }   
   
}// Implementation of aruco_callback
/**
 * @brief Callback function for camera1 image data.
 * 
 * @param msg Camera1 image message.
 */
void TheNode::cam1_callback(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg) {

            geometry_msgs::msg::Pose part_pose = msg->part_poses[0].pose;
            geometry_msgs::msg::PoseStamped part_pose_stamped;

            part_pose_stamped.pose = part_pose;
            part_pose_stamped.header.frame_id = "camera1_frame";

            part1_type = msg->part_poses[0].part.type;
            part1_color = msg->part_poses[0].part.color;

            part1_pose_wframe.header.frame_id = "world";


            geometry_msgs::msg::TransformStamped transform_stamped = tf_buffer_->lookupTransform(
                "world", "camera1_frame", tf2::TimePointZero);

            
            tf2::doTransform(part_pose_stamped, part1_pose_wframe, transform_stamped);


            RCLCPP_INFO(this->get_logger(), "Part in camera1 frame: [part1_type: %d, part1_color: %d]", part1_type, part1_color);

            RCLCPP_INFO(this->get_logger(), "Transformed Marker Pose in odom frame: [x: %f, y: %f, z: %f]",
                        part1_pose_wframe.pose.position.x, part1_pose_wframe.pose.position.y, part1_pose_wframe.pose.position.z);

            cam1_received_ = true;
            check_and_send_goal(); 

            if(cam1_received_){subscription_cam1.reset();}      
    
}// Implementation of cam1_callback
/**
 * @brief Callback function for camera2 image data.
 * 
 * @param msg Camera2 image message.
 */
void TheNode::cam2_callback(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg) {


            geometry_msgs::msg::Pose part_pose = msg->part_poses[0].pose;
            geometry_msgs::msg::PoseStamped part_pose_stamped;

            part_pose_stamped.pose = part_pose;
            part_pose_stamped.header.frame_id = "camera2_frame";

            part2_type = msg->part_poses[0].part.type;
            part2_color = msg->part_poses[0].part.color;

            part2_pose_wframe.header.frame_id = "world";


            geometry_msgs::msg::TransformStamped transform_stamped = tf_buffer_->lookupTransform(
                "world", "camera2_frame", tf2::TimePointZero);

            
            tf2::doTransform(part_pose_stamped, part2_pose_wframe, transform_stamped);


            RCLCPP_INFO(this->get_logger(), "Part in camera2 frame: [part2_type: %d, part2_color: %d]", part2_type, part2_color);


            RCLCPP_INFO(this->get_logger(), "Transformed Marker Pose in odom frame: [x: %f, y: %f, z: %f]",
                        part2_pose_wframe.pose.position.x, part2_pose_wframe.pose.position.y, part2_pose_wframe.pose.position.z);
           
            cam2_received_ = true;
            check_and_send_goal();

            if(cam2_received_){subscription_cam2.reset();}   
    
}// Implementation of cam2_callback
/**
 * @brief Callback function for camera3 image data.
 * 
 * @param msg Camera3 image message.
 */

void TheNode::cam3_callback(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg) {

                
            geometry_msgs::msg::Pose part_pose = msg->part_poses[0].pose;
            geometry_msgs::msg::PoseStamped part_pose_stamped;

            part_pose_stamped.pose = part_pose;
            part_pose_stamped.header.frame_id = "camera3_frame";

            part3_type = msg->part_poses[0].part.type;
            part3_color = msg->part_poses[0].part.color;

            part3_pose_wframe.header.frame_id = "world";


            geometry_msgs::msg::TransformStamped transform_stamped = tf_buffer_->lookupTransform(
                "world", "camera3_frame", tf2::TimePointZero);

            
            tf2::doTransform(part_pose_stamped, part3_pose_wframe, transform_stamped);


            RCLCPP_INFO(this->get_logger(), "Part in camera3 frame: [part3_type: %d, part3_color: %d]", part3_type, part3_color);


            RCLCPP_INFO(this->get_logger(), "Transformed Marker Pose in odom frame: [x: %f, y: %f, z: %f]",
                        part3_pose_wframe.pose.position.x, part3_pose_wframe.pose.position.y, part3_pose_wframe.pose.position.z);
           
            cam3_received_ = true;
            check_and_send_goal();

            if(cam3_received_){subscription_cam3.reset();}   
    
}// Implementation of cam3_callback

/**
 * @brief Callback function for camera4 image data.
 * 
 * @param msg Camera4 image message.
 */
void TheNode::cam4_callback(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg) {
                geometry_msgs::msg::Pose part_pose = msg->part_poses[0].pose;
            geometry_msgs::msg::PoseStamped part_pose_stamped;

            part_pose_stamped.pose = part_pose;
            part_pose_stamped.header.frame_id = "camera4_frame";

            part4_type = msg->part_poses[0].part.type;
            part4_color = msg->part_poses[0].part.color;

            part4_pose_wframe.header.frame_id = "world";

            geometry_msgs::msg::TransformStamped transform_stamped = tf_buffer_->lookupTransform(
                "world", "camera4_frame", tf2::TimePointZero);

            
            tf2::doTransform(part_pose_stamped, part4_pose_wframe, transform_stamped);


            RCLCPP_INFO(this->get_logger(), "Part in camera4 frame: [part4_type: %d, part4_color: %d]", part4_type, part4_color);


            RCLCPP_INFO(this->get_logger(), "Transformed Marker Pose in odom frame: [x: %f, y: %f, z: %f]",
                        part4_pose_wframe.pose.position.x, part4_pose_wframe.pose.position.y, part4_pose_wframe.pose.position.z);

            cam4_received_ = true;
            check_and_send_goal();

            if(cam4_received_){subscription_cam4.reset();}  
  
}// Implementation of cam4_callback

/**
 * @brief Callback function for camera5 image data.
 * 
 * @param msg Camera5 image message.
 */
void TheNode::cam5_callback(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg) {
            geometry_msgs::msg::Pose part_pose = msg->part_poses[0].pose;
            geometry_msgs::msg::PoseStamped part_pose_stamped;

            part_pose_stamped.pose = part_pose;
            part_pose_stamped.header.frame_id = "camera5_frame";

            part5_type = msg->part_poses[0].part.type;
            part5_color = msg->part_poses[0].part.color;

            part5_pose_wframe.header.frame_id = "world";


            geometry_msgs::msg::TransformStamped transform_stamped = tf_buffer_->lookupTransform(
                "world", "camera5_frame", tf2::TimePointZero);

            
            tf2::doTransform(part_pose_stamped, part5_pose_wframe, transform_stamped);


            RCLCPP_INFO(this->get_logger(), "Part in camera5 frame: [part5_type: %d, part5_color: %d]", part5_type, part5_color);


            RCLCPP_INFO(this->get_logger(), "Transformed Marker Pose in odom frame: [x: %f, y: %f, z: %f]",
                        part5_pose_wframe.pose.position.x, part5_pose_wframe.pose.position.y, part5_pose_wframe.pose.position.z);

            cam5_received_ = true;
            check_and_send_goal();

            if(cam5_received_){subscription_cam5.reset();}    

}// Implementation of cam5_callback

/**
 * @brief Method to check conditions and send a goal to the navigation action server.
 */
void TheNode::check_and_send_goal() {
            if (cam1_received_ && cam2_received_ && cam3_received_ && cam4_received_ && cam5_received_ && aruco_received_) {
                set_initial_pose();
                std::this_thread::sleep_for(std::chrono::seconds(5));                
                send_goal(); // Call send_goal() only when all callbacks have received data
            }

}

/**
 * @brief Method to send a goal to the navigation action server.
 */
void TheNode::send_goal() {

      using namespace std::placeholders;

        if (!this->client_->wait_for_action_server()) {
        RCLCPP_ERROR(this->get_logger(),
                        "Action server not available after waiting");
        rclcpp::shutdown();
        }

        std::map<std::string, uint8_t> colorMap = {
            {"red", mage_msgs::msg::Part::RED},
            {"green", mage_msgs::msg::Part::GREEN},
            {"blue", mage_msgs::msg::Part::BLUE},
            {"orange", mage_msgs::msg::Part::ORANGE},
            {"purple", mage_msgs::msg::Part::PURPLE}
        };

        std::map<std::string, uint8_t> typeMap = {
            {"battery", mage_msgs::msg::Part::BATTERY},
            {"pump", mage_msgs::msg::Part::PUMP},
            {"sensor", mage_msgs::msg::Part::SENSOR},
            {"regulator", mage_msgs::msg::Part::REGULATOR}
        };

        mage_msgs::msg::Part wp1, wp2, wp3, wp4, wp5;
        std::string the_aruco = "aruco_" + std::to_string(marker_id);

        wp1.color = colorMap[config["theNode"]["ros__parameters"][the_aruco]["wp1"]["color"].as<std::string>().c_str()];
        wp1.type = typeMap[config["theNode"]["ros__parameters"][the_aruco]["wp1"]["type"].as<std::string>().c_str()];

        // Populate wp2
        wp2.color = colorMap[config["theNode"]["ros__parameters"][the_aruco]["wp2"]["color"].as<std::string>().c_str()];
        wp2.type = typeMap[config["theNode"]["ros__parameters"][the_aruco]["wp2"]["type"].as<std::string>().c_str()];

        // Populate wp3
        wp3.color = colorMap[config["theNode"]["ros__parameters"][the_aruco]["wp3"]["color"].as<std::string>().c_str()];
        wp3.type = typeMap[config["theNode"]["ros__parameters"][the_aruco]["wp3"]["type"].as<std::string>().c_str()];

        // Populate wp4
        wp4.color = colorMap[config["theNode"]["ros__parameters"][the_aruco]["wp4"]["color"].as<std::string>().c_str()];
        wp4.type = typeMap[config["theNode"]["ros__parameters"][the_aruco]["wp4"]["type"].as<std::string>().c_str()];

        // Populate wp5
        wp5.color = colorMap[config["theNode"]["ros__parameters"][the_aruco]["wp5"]["color"].as<std::string>().c_str()];
        wp5.type = typeMap[config["theNode"]["ros__parameters"][the_aruco]["wp5"]["type"].as<std::string>().c_str()];


        //working...
        // std::string type = config["theNode"]["ros__parameters"]["aruco_0"]["wp1"]["color"].as<std::string>();  // Get the 'type' value
        // std::string color = config["theNode"]["ros__parameters"]["aruco_0"]["wp1"]["type"].as<std::string>();  // Get the 'color' value


        // RCLCPP_INFO(this->get_logger(), "duhhh: %d",
        //     wp5.type);


        /*
        Loop here to check which of the part correspond to wp1 through wp5 and based on that, we give them below. 

        Something like, wp1.color== part1_color && wp1.type == part1_color then 
        */

        if (wp1.type == part1_type && wp1.color == part1_color) {
            wp1_pose_wframe = part1_pose_wframe;
        } else if (wp1.type == part2_type && wp1.color == part2_color) {
            wp1_pose_wframe = part2_pose_wframe;
        } else if (wp1.type == part3_type && wp1.color == part3_color) {
            wp1_pose_wframe = part3_pose_wframe;
        } else if (wp1.type == part4_type && wp1.color == part4_color) {
            wp1_pose_wframe = part4_pose_wframe;
        } else if (wp1.type == part5_type && wp1.color == part5_color) {
            wp1_pose_wframe = part5_pose_wframe; 
        }

        if (wp2.type == part1_type && wp2.color == part1_color) {
            wp2_pose_wframe = part1_pose_wframe;
        } else if (wp2.type == part2_type && wp2.color == part2_color) {
            wp2_pose_wframe = part2_pose_wframe;
        } else if (wp2.type == part3_type && wp2.color == part3_color) {
            wp2_pose_wframe = part3_pose_wframe;
        } else if (wp2.type == part4_type && wp2.color == part4_color) {
            wp2_pose_wframe = part4_pose_wframe;
        } else if (wp2.type == part5_type && wp2.color == part5_color) {
            wp2_pose_wframe = part5_pose_wframe;
        }

        if (wp3.type == part1_type && wp3.color == part1_color) {
            wp3_pose_wframe = part1_pose_wframe;
        } else if (wp3.type == part2_type && wp3.color == part2_color) {
            wp3_pose_wframe = part2_pose_wframe;
        } else if (wp3.type == part3_type && wp3.color == part3_color) {
            wp3_pose_wframe = part3_pose_wframe;
        } else if (wp3.type == part4_type && wp3.color == part4_color) {
            wp3_pose_wframe = part4_pose_wframe;
        } else if (wp3.type == part5_type && wp3.color == part5_color) {
            wp3_pose_wframe = part5_pose_wframe;
        }

        if (wp4.type == part1_type && wp4.color == part1_color) {
            wp4_pose_wframe = part1_pose_wframe;
        } else if (wp4.type == part2_type && wp4.color == part2_color) {
            wp4_pose_wframe = part2_pose_wframe;
        } else if (wp4.type == part3_type && wp4.color == part3_color) {
            wp4_pose_wframe = part3_pose_wframe;
        } else if (wp4.type == part4_type && wp4.color == part4_color) {
            wp4_pose_wframe = part4_pose_wframe;
        } else if (wp4.type == part5_type && wp4.color == part5_color) {
            wp4_pose_wframe = part5_pose_wframe;
        }

        if (wp5.type == part1_type && wp5.color == part1_color) {
            wp5_pose_wframe = part1_pose_wframe;
        } else if (wp5.type == part2_type && wp5.color == part2_color) {
            wp5_pose_wframe = part2_pose_wframe;
        } else if (wp5.type == part3_type && wp5.color == part3_color) {
            wp5_pose_wframe = part3_pose_wframe;
        } else if (wp5.type == part4_type && wp5.color == part4_color) {
            wp5_pose_wframe = part4_pose_wframe;
        } else if (wp5.type == part5_type && wp5.color == part5_color) {
            wp5_pose_wframe = part5_pose_wframe;
        }



        // Create an array to store the pose stamped variables
        std::array<geometry_msgs::msg::PoseStamped, 5> waypoints = {
            wp1_pose_wframe, wp2_pose_wframe, wp3_pose_wframe, wp4_pose_wframe, wp5_pose_wframe
        };

            // Create a goal message
        auto goal_msg = nav2_msgs::action::FollowWaypoints::Goal();
        goal_msg.poses.reserve(waypoints.size()); // Reserve space for all waypoints

        // Add each pose stamped variable to the goal poses
        for (const auto& waypoint : waypoints) {
            goal_msg.poses.push_back(waypoint);
        }

        RCLCPP_INFO(this->get_logger(), "Sending goal");

        RCLCPP_INFO(this->get_logger(), "Transformed Marker Pose in odom frame: [x: %f, y: %f, z: %f]",
            part1_pose_wframe.pose.position.x, part1_pose_wframe.pose.position.y, part1_pose_wframe.pose.position.z);

                    RCLCPP_INFO(this->get_logger(), "Transformed Marker Pose in odom frame: [x: %f, y: %f, z: %f]",
        part2_pose_wframe.pose.position.x, part2_pose_wframe.pose.position.y, part2_pose_wframe.pose.position.z);

                    RCLCPP_INFO(this->get_logger(), "Transformed Marker Pose in odom frame: [x: %f, y: %f, z: %f]",
        part3_pose_wframe.pose.position.x, part3_pose_wframe.pose.position.y, part3_pose_wframe.pose.position.z);

        RCLCPP_INFO(this->get_logger(), "Transformed Marker Pose in odom frame: [x: %f, y: %f, z: %f]",
            part4_pose_wframe.pose.position.x, part4_pose_wframe.pose.position.y, part4_pose_wframe.pose.position.z);

        RCLCPP_INFO(this->get_logger(), "Testing to see if its inserting garbage: [x: %f, y: %f, z: %f]",
                    part5_pose_wframe.pose.position.x, part5_pose_wframe.pose.position.y, part5_pose_wframe.pose.position.z);

        auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::FollowWaypoints>::SendGoalOptions();
        client_->async_send_goal(goal_msg, send_goal_options);

        //The frame id for each of these
        //Prof mentioned that they needs to be in "map" but I am arriving there through "world"
        //Should I mannually change it to map? I mean, what exactly is the difference between map and world?
}

/**
 * @brief Method to set the initial pose for navigation.
 */
void TheNode::set_initial_pose() {
            auto message = geometry_msgs::msg::PoseWithCovarianceStamped();

        tf2::Quaternion q;
        q.setRPY(0, 0, -1.5708);

        message.header.frame_id = "map";
        message.pose.pose.position.x = 1.000000;
        message.pose.pose.position.y = -1.590593;
        message.pose.pose.position.z = 0.007828;
        message.pose.pose.orientation.x = q.x();
        message.pose.pose.orientation.y = q.y();
        message.pose.pose.orientation.z = q.z();
        message.pose.pose.orientation.w = q.w();
        initial_pose_pub_->publish(message);
    
}

/**
 * @brief Main function for the node.
 * 
 * @param argc Number of arguments.
 * @param argv Argument vector.
 * @return int Return code.
 */
int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TheNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
