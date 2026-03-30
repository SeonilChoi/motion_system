#ifndef MOTOR_MANAGER_NODE_HPP_
#define MOTOR_MANAGER_NODE_HPP_

#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "motion_system_msgs/msg/motor_status.hpp"

#include "motor_manager/motor_manager.hpp"

class MotorManagerNode : public rclcpp::Node {
public:
    using MotorStatus = motion_system_msgs::msg::MotorStatus;

    explicit MotorManagerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    ~MotorManagerNode();

private:
    void motor_command_callback(const MotorStatus::SharedPtr msg);

    void timer_callback();

    rclcpp::Subscription<MotorStatus>::SharedPtr motor_command_subscriber_;

    rclcpp::Publisher<MotorStatus>::SharedPtr motor_status_publisher_;

    rclcpp::TimerBase::SharedPtr motor_status_timer_;

    std::string config_file_;

    std::unique_ptr<motor_manager::MotorManager> motor_manager_;

    std::thread manager_run_thread_;
};

#endif // MOTOR_MANAGER_NODE_HPP_
