#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class ArmControllerNode : public rclcpp::Node
{
public:
  ArmControllerNode()
  : Node("arm_controller_node")
  {
    // Publisher per il topic /position_controller/commands
    command_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/position_controller/commands", 10);

    // Subscriber per il topic joint_states
    joint_state_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", 10, std::bind(&ArmControllerNode::jointStateCallback, this, _1));

    // Timer per inviare il comando una sola volta dopo l'inizio
    timer_ = this->create_wall_timer(
      5000ms, std::bind(&ArmControllerNode::publishCommand, this));

    RCLCPP_INFO(this->get_logger(), "Arm Controller Node initialized. Waiting for joint states...");
  }

private:
  // Funzione callback per il subscriber del topic joint_states
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Current joint positions:");
    for (size_t i = 0; i < msg->position.size(); ++i) {
      RCLCPP_INFO(this->get_logger(), "  Joint %zu position: %f", i, msg->position[i]);
    }
  }

  // Funzione per pubblicare il comando sul topic /position_controller/commands
  void publishCommand()
  {
    auto command_msg = std_msgs::msg::Float64MultiArray();
    command_msg.data = {1.0, 1.0, 1.0, 1.0}; 
    RCLCPP_INFO(this->get_logger(), "Publishing command to position (1, 1, 1, 1)");

    command_publisher_->publish(command_msg);

    // Ferma il timer dopo aver inviato il comando una volta
    timer_->cancel();
  }

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr command_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscriber_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArmControllerNode>());
  rclcpp::shutdown();
  return 0;
}


