#pragma once

#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace navigation
{

    /**
     * @brief A class representing a navigation demo node.
     */
    class NavigationDemo : public rclcpp::Node
    {
    public:
        using NavigateToPose = nav2_msgs::action::NavigateToPose;
        using GoalHandleNavigation = rclcpp_action::ClientGoalHandle<NavigateToPose>;

        /**
         * @brief Constructor for NavigationDemo class.
         * @param node_name The name of the node.
         */
        NavigationDemo(std::string node_name) : Node(node_name)
        {
            // Initialize the action client
            client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

            // Initialize the initial pose publisher
            initial_pose_pub_ =
                this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
                    "initialpose", 10);

            // Set the initial pose for navigation
            set_initial_pose();

            // Pause for 5 seconds
            std::this_thread::sleep_for(std::chrono::seconds(5));

            // Send the goal
            send_goal();
        }

    private:
        /**
         * @brief Publisher to the topic /initialpose.
         */
        rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
            initial_pose_pub_;

        /**
         * @brief Action client for the action server navigate_to_pose.
         */
        rclcpp_action::Client<NavigateToPose>::SharedPtr client_;

        /**
         * @brief Callback function for handling the response from the server after sending the goal.
         * @param future A shared future representing the goal handle.
         */
        void goal_response_callback(
            std::shared_future<GoalHandleNavigation::SharedPtr> future);

        /**
         * @brief Callback function for handling feedback while the robot is driving towards the goal.
         * @param feedback The feedback received from the server.
         */
        void feedback_callback(
            GoalHandleNavigation::SharedPtr,
            const std::shared_ptr<const NavigateToPose::Feedback> feedback);

        /**
         * @brief Callback function for handling the result after the action has completed.
         * @param result The result of the action.
         */
        void result_callback(const GoalHandleNavigation::WrappedResult &result);

        /**
         * @brief Method to build and send a goal using the action client.
         */
        void send_goal();

        /**
         * @brief Method to set the initial pose for navigation.
         */
        void set_initial_pose();
    }; // class NavigationDemo

} // namespace navigation
