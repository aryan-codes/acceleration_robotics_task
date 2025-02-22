// turtlebot_pose_tracker.cpp
#include <memory>
#include <string>
#include <iostream>
#include <geometry_msgs/msg/pose.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using std::placeholders::_1;
using std::placeholders::_2;
using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

class TurtlebotPoseTracker : public rclcpp::Node
{
public:
  TurtlebotPoseTracker() : Node("turtlebot_pose_tracker")
  {
    // Initialize TF listener
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Initialize navigation action client
    nav_action_client_ = rclcpp_action::create_client<NavigateToPose>(
      this, "navigate_to_pose");

    // Use a timer to check if the action server is available and to print initial pose
    timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&TurtlebotPoseTracker::init_callback, this));
  }

private:
  // Timer callback for initialization
  void init_callback()
  {
    if (!nav_action_client_->wait_for_action_server(std::chrono::milliseconds(100))) {
      RCLCPP_INFO(this->get_logger(), "Waiting for navigation action server...");
      return;
    }
    
    // Cancel the timer once we're ready
    timer_->cancel();
    
    // Print initial pose
    print_current_pose();
    
    // Ask for the first goal
    prompt_for_goal();
  }

  // Print current robot position
  void print_current_pose()
  {
    try {
      geometry_msgs::msg::TransformStamped transform = 
        tf_buffer_->lookupTransform("map", "base_footprint", tf2::TimePointZero);
      
      double x = transform.transform.translation.x;
      double y = transform.transform.translation.y;
      
      // Convert quaternion to Euler angles
      tf2::Quaternion q(
        transform.transform.rotation.x,
        transform.transform.rotation.y,
        transform.transform.rotation.z,
        transform.transform.rotation.w);
      tf2::Matrix3x3 m(q);
      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);
      
      RCLCPP_INFO(this->get_logger(), "Current robot pose: x=%.2f, y=%.2f, yaw=%.2f", x, y, yaw);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_ERROR(this->get_logger(), "Could not get robot pose: %s", ex.what());
    }
  }

  // Prompt the user for a goal pose
  void prompt_for_goal()
  {
    float x, y, theta;
    std::cout << "\nEnter goal pose (x y theta): ";
    std::cin >> x >> y >> theta;
    
    // Create the goal message
    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose.header.frame_id = "map";
    goal_msg.pose.header.stamp = this->get_clock()->now();
    goal_msg.pose.pose.position.x = x;
    goal_msg.pose.pose.position.y = y;
    
    // Convert theta to quaternion
    tf2::Quaternion q;
    q.setRPY(0, 0, theta);
    goal_msg.pose.pose.orientation.x = q.x();
    goal_msg.pose.pose.orientation.y = q.y();
    goal_msg.pose.pose.orientation.z = q.z();
    goal_msg.pose.pose.orientation.w = q.w();
    
    RCLCPP_INFO(this->get_logger(), "Navigating to: x=%.2f, y=%.2f, theta=%.2f", x, y, theta);
    
    // Send the goal
    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      std::bind(&TurtlebotPoseTracker::goal_response_callback, this, _1);
    send_goal_options.feedback_callback =
      std::bind(&TurtlebotPoseTracker::feedback_callback, this, _1, _2);
    send_goal_options.result_callback =
      std::bind(&TurtlebotPoseTracker::result_callback, this, _1);
    
    nav_action_client_->async_send_goal(goal_msg, send_goal_options);
  }
  
  // Goal response callback
  void goal_response_callback(const GoalHandleNavigateToPose::SharedPtr & goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by the action server");
      // Ask for a new goal if the previous one was rejected
      prompt_for_goal();
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by the action server");
    }
  }
  
  // Feedback callback
  void feedback_callback(
    GoalHandleNavigateToPose::SharedPtr,
    const std::shared_ptr<const NavigateToPose::Feedback> feedback)
  {
    // We could print distance remaining or other feedback here
    (void)feedback; // Suppress unused parameter warning
  }
  
  // Result callback
  void result_callback(const GoalHandleNavigateToPose::WrappedResult & result)
  {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(this->get_logger(), "Reached goal position!");
        std::cout << "\n*** REACHED ***\n" << std::endl;
        
        // Print the final pose after reaching the goal
        print_current_pose();
        
        // Ask for the next goal
        prompt_for_goal();
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
        prompt_for_goal();
        break;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
        prompt_for_goal();
        break;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        prompt_for_goal();
        break;
    }
  }

  // Class members
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_action_client_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TurtlebotPoseTracker>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}