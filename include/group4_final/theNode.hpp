#pragma once

#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "ros2_aruco_interfaces/msg/aruco_markers.hpp"
#include "mage_msgs/msg/advanced_logical_camera_image.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "mage_msgs/msg/part.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav2_msgs/action/follow_waypoints.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

/**
 * @brief A class representing a node with multiple camera and marker callbacks.
 */
class TheNode : public rclcpp::Node
{
public:
    /**
     * @brief Constructor for TheNode class.
     */
    TheNode();

private:
    bool cam1_received_;  /**< Flag to indicate whether data from camera 1 has been received. */
    bool cam2_received_;  /**< Flag to indicate whether data from camera 2 has been received. */
    bool cam3_received_;  /**< Flag to indicate whether data from camera 3 has been received. */
    bool cam4_received_;  /**< Flag to indicate whether data from camera 4 has been received. */
    bool cam5_received_;  /**< Flag to indicate whether data from camera 5 has been received. */
    bool aruco_received_; /**< Flag to indicate whether Aruco marker data has been received. */

    /**
     * @brief Callback function for processing Aruco marker messages.
     * @param msg The Aruco marker message.
     */
    void aruco_callback(const ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr msg);

    /**
     * @brief Callback function for processing camera 1 messages.
     * @param msg The camera 1 message.
     */
    void cam1_callback(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg);

    /**
     * @brief Callback function for processing camera 2 messages.
     * @param msg The camera 2 message.
     */
    void cam2_callback(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg);

    /**
     * @brief Callback function for processing camera 3 messages.
     * @param msg The camera 3 message.
     */
    void cam3_callback(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg);

    /**
     * @brief Callback function for processing camera 4 messages.
     * @param msg The camera 4 message.
     */
    void cam4_callback(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg);

    /**
     * @brief Callback function for processing camera 5 messages.
     * @param msg The camera 5 message.
     */
    void cam5_callback(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg);

    /**
     * @brief Method to check conditions and send a goal to the navigation action server.
     */
    void check_and_send_goal();

    /**
     * @brief Method to send a goal to the navigation action server.
     */
    void send_goal();

    /**
     * @brief Method to set the initial pose for navigation.
     */
    void set_initial_pose();

    rclcpp::Subscription<ros2_aruco_interfaces::msg::ArucoMarkers>::SharedPtr subscription_aruco;  /**< Subscription for Aruco marker messages. */
    rclcpp::Subscription<mage_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr subscription_cam1; /**< Subscription for camera 1 messages. */
    rclcpp::Subscription<mage_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr subscription_cam2; /**< Subscription for camera 2 messages. */
    rclcpp::Subscription<mage_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr subscription_cam3; /**< Subscription for camera 3 messages. */
    rclcpp::Subscription<mage_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr subscription_cam4; /**< Subscription for camera 4 messages. */
    rclcpp::Subscription<mage_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr subscription_cam5; /**< Subscription for camera 5 messages. */

    std::string file_path; /**< File path for configuration files. */
    YAML::Node config;     /**< YAML configuration node. */
    int marker_id;         /**< Marker ID for Aruco marker. */

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;              /**< Shared pointer to TF2 buffer. */
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_; /**< Shared pointer to TF2 transform listener. */

    int part1_color, part2_color, part3_color, part4_color, part5_color; /**< Colors for different parts. */
    int part1_type, part2_type, part3_type, part4_type, part5_type;      /**< Types for different parts. */

    geometry_msgs::msg::PoseStamped part1_pose_wframe; /**< Pose of part 1 in the world frame. */
    geometry_msgs::msg::PoseStamped part2_pose_wframe; /**< Pose of part 2 in the world frame. */
    geometry_msgs::msg::PoseStamped part3_pose_wframe; /**< Pose of part 3 in the world frame. */
    geometry_msgs::msg::PoseStamped part4_pose_wframe; /**< Pose of part 4 in the world frame. */
    geometry_msgs::msg::PoseStamped part5_pose_wframe; /**< Pose of part 5 in the world frame. */

    geometry_msgs::msg::PoseStamped wp1_pose_wframe; /**< Pose of waypoint 1 in the world frame. */
    geometry_msgs::msg::PoseStamped wp2_pose_wframe; /**< Pose of waypoint 2 in the world frame. */
    geometry_msgs::msg::PoseStamped wp3_pose_wframe; /**< Pose of waypoint 3 in the world frame. */
    geometry_msgs::msg::PoseStamped wp4_pose_wframe; /**< Pose of waypoint 4 in the world frame. */
    geometry_msgs::msg::PoseStamped wp5_pose_wframe; /**< Pose of waypoint 5 in the world frame. */

    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_pub_; /**< Publisher for initial pose. */
    rclcpp_action::Client<nav2_msgs::action::FollowWaypoints>::SharedPtr client_;                  /**< Action client for navigation action server. */
    std::string aruco_;                                                                            /**< Aruco marker string. */

    // void send_goal();
};
