#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class TrajToJointStateBridge : public rclcpp::Node
{
public:
  TrajToJointStateBridge()
  : Node("traj_to_jointstate_bridge")
  {
    traj_sub_ = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(
      "/arm_controller/joint_trajectory", 10,
      std::bind(&TrajToJointStateBridge::trajectory_callback, this, _1));

    joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
    RCLCPP_INFO(this->get_logger(), "Trajectory → JointState bridge started.");
  }

private:
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr traj_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;

  void trajectory_callback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
  {
    rclcpp::Time start_time = this->now();

    for (const auto & point : msg->points)
    {
      auto now = this->now();
      rclcpp::Duration sleep_time = rclcpp::Duration(point.time_from_start) - (now - start_time);
      if (sleep_time > rclcpp::Duration::from_seconds(0))
      {
        rclcpp::sleep_for(std::chrono::nanoseconds(sleep_time.nanoseconds()));
      }

      sensor_msgs::msg::JointState js_msg;
      js_msg.header.stamp = this->now();
      js_msg.name = msg->joint_names;
      js_msg.position = point.positions;

      // 可选添加 velocity / effort，如果你需要
      if (!point.velocities.empty()) js_msg.velocity = point.velocities;
      if (!point.effort.empty()) js_msg.effort = point.effort;

      joint_state_pub_->publish(js_msg);
    }

    RCLCPP_INFO(this->get_logger(), "Trajectory executed and converted to JointState.");
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrajToJointStateBridge>());
  rclcpp::shutdown();
  return 0;
}
