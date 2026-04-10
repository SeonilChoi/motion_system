#include <algorithm>
#include <chrono>
#include <memory>
#include <stdexcept>

#include "common_motor_interface/motor_frame.hpp"

#include "motion_system_pkg/motor_manager_node.hpp"

MotorManagerNode::MotorManagerNode(const rclcpp::NodeOptions& options)
    : Node("motor_manager_node", options)
{
    motor_command_subscriber_ = this->create_subscription<MotorStatus>(
        "motor_command", rclcpp::QoS(1).best_effort(),
        [this](const MotorStatus::SharedPtr msg) {
            motor_command_callback(msg);
        }
    );

    user_command_subscriber_ = this->create_subscription<Empty>(
        "user_command", rclcpp::QoS(1).best_effort(),
        [this](const Empty::SharedPtr msg) {
            user_command_callback(msg);
        }
    );

    motor_status_publisher_ = this->create_publisher<MotorStatus>(
        "motor_state", rclcpp::QoS(1).best_effort()
    );

    motor_status_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1),
        [this]() {
            timer_callback();
        }
    );

    config_file_ = this->declare_parameter<std::string>("config_file", "");
    if (config_file_.empty()) {
        throw std::runtime_error(
            "Parameter 'config_file' is empty. Use e.g. "
            "`ros2 launch motion_system_pkg motion_system.launch.py`.");
    }

    motor_manager_ = std::make_unique<motor_manager::MotorManager>(config_file_);

    manager_run_thread_ = std::thread([this]() {
        try {
            motor_manager_->run();
        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "MotorManager::run() failed: %s", e.what());
        }
    });
}

MotorManagerNode::~MotorManagerNode()
{
    if (motor_manager_) {
        motor_manager_->request_exit();
    }
    if (manager_run_thread_.joinable()) {
        manager_run_thread_.join();
    }
}

void MotorManagerNode::motor_command_callback(const MotorStatus::SharedPtr msg)
{
    const size_t size = msg->controller_index.size();
    
    motor_interface::motor_frame_t motor_frame[motor_manager::MAX_CONTROLLER_SIZE] = {};

    for (uint8_t i = 0; i < size; i++) {
        motor_frame[i].number_of_target_interfaces = msg->number_of_target_interfaces[i];

        const uint8_t n_if = motor_frame[i].number_of_target_interfaces;
        const size_t vec_n = msg->target_interface_id[i].data.size();
        const uint8_t copy_n = static_cast<uint8_t>(std::min(
            static_cast<size_t>(n_if),
            vec_n));
        for (uint8_t j = 0; j < copy_n; j++) {
            motor_frame[i].target_interface_id[j] = msg->target_interface_id[i].data[j];
        }
        motor_frame[i].controller_index = msg->controller_index[i];
        motor_frame[i].controlword = msg->controlword[i];
        motor_frame[i].statusword = msg->statusword[i];
        motor_frame[i].errorcode = msg->errorcode[i];
        motor_frame[i].position = msg->position[i];
        motor_frame[i].velocity = msg->velocity[i];
        motor_frame[i].torque = msg->torque[i];
    }

    motor_manager_->write(motor_frame, size);
}

void MotorManagerNode::user_command_callback(const Empty::SharedPtr msg)
{
    motor_manager_->request_stop();
}

void MotorManagerNode::timer_callback()
{
    const uint8_t n = motor_manager_->number_of_controllers();
    if (n == 0) {
        return;
    }

    motor_interface::motor_frame_t status[motor_manager::MAX_CONTROLLER_SIZE] = {};
    motor_manager_->read(status);

    MotorStatus msg;
    msg.number_of_target_interfaces.resize(n);
    msg.controller_index.resize(n);
    msg.controlword.resize(n);
    msg.statusword.resize(n);
    msg.errorcode.resize(n);
    msg.position.resize(n);
    msg.velocity.resize(n);
    msg.torque.resize(n);

    for (uint8_t i = 0; i < n; i++) {
        msg.controller_index[i] = status[i].controller_index;
        msg.controlword[i] = status[i].controlword;
        msg.statusword[i] = status[i].statusword;
        msg.errorcode[i] = status[i].errorcode;
        msg.position[i] = status[i].position;
        msg.velocity[i] = status[i].velocity;
        msg.torque[i] = status[i].torque;
    }

    motor_status_publisher_->publish(msg);
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotorManagerNode>());
    rclcpp::shutdown();
    return 0;
}
