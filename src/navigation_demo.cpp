#include "navigation_demo.hpp"

namespace navigation {

/**
 * @brief Sets the initial pose for navigation.
 */
void NavigationDemo::set_initial_pose() {
  auto message = geometry_msgs::msg::PoseWithCovarianceStamped();
  message.header.frame_id = "map";
  message.pose.pose.position.x = -2.0;
  message.pose.pose.position.y = -0.5;
  message.pose.pose.orientation.x = 0.0;
  message.pose.pose.orientation.y = 0.0;
  message.pose.pose.orientation.z = 0.0;
  message.pose.pose.orientation.w = 1.0;
  initial_pose_pub_->publish(message);
}

/**
 * @brief Sends a goal to the navigation action server.
 */
void NavigationDemo::send_goal() {
  using namespace std::placeholders;

  if (!this->client_->wait_for_action_server()) {
    RCLCPP_ERROR(this->get_logger(),
                 "Action server not available after waiting");
    rclcpp::shutdown();
  }

  auto goal_msg = NavigateToPose::Goal();
  goal_msg.pose.header.frame_id = "map";
  goal_msg.pose.pose.position.x = 1.5;
  goal_msg.pose.pose.position.y = 0.0;
  goal_msg.pose.pose.position.z = 0.0;
  goal_msg.pose.pose.orientation.x = 0.0;
  goal_msg.pose.pose.orientation.y = 0.0;
  goal_msg.pose.pose.orientation.z = 0.0;
  goal_msg.pose.pose.orientation.w = 0.7787;

  RCLCPP_INFO(this->get_logger(), "Sending goal");

  auto send_goal_options =
      rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
  send_goal_options.goal_response_callback =
      std::bind(&NavigationDemo::goal_response_callback, this, _1);
  // send_goal_options.feedback_callback =
  //     std::bind(&NavigationDemo::feedback_callback, this, _1, _2);
  send_goal_options.result_callback =
      std::bind(&NavigationDemo::result_callback, this, _1);

  client_->async_send_goal(goal_msg, send_goal_options);
}

/**
 * @brief Callback function for handling the response from the server after sending the goal.
 * @param future A shared future representing the goal handle.
 */
void NavigationDemo::goal_response_callback(
    std::shared_future<GoalHandleNavigation::SharedPtr> future) {
  auto goal_handle = future.get();
  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    rclcpp::shutdown();
  } else {
    RCLCPP_INFO(this->get_logger(),
                "Goal accepted by server, waiting for result");
  }
}

/**
 * @brief Callback function for handling feedback while the robot is driving towards the goal.
 * @param feedback The feedback received from the server.
 */
void NavigationDemo::feedback_callback(
    GoalHandleNavigation::SharedPtr,
    const std::shared_ptr<const NavigateToPose::Feedback> feedback) {
  RCLCPP_INFO(this->get_logger(), "Robot is driving towards the goal");
}

/**
 * @brief Callback function for handling the result after the action has completed.
 * @param result The result of the action.
 */
void NavigationDemo::result_callback(
    const GoalHandleNavigation::WrappedResult& result) {
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
      return;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
      return;
    default:
      RCLCPP_ERROR(this->get_logger(), "Unknown result code");
      return;
  }
  rclcpp::shutdown();
}

}  // namespace navigation

/**
 * @brief The main function to initialize and run the navigation demo node.
 */
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<navigation::NavigationDemo>("navigation_demo");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
