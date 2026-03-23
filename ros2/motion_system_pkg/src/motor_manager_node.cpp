#include <algorithm>
#include <chrono>
#include <memory>
#include <stdexcept>

#include "common_motor_interface/motor_frame.hpp"

#include "motion_system_pkg/motor_manager_node.hpp"

MotorManagerNode::MotorManagerNode(const rclcpp::NodeOptions& options)
    : Node("motor_manager_node", options)
{
    motor_command_subscriber_ = this->create_subscription<MotorFrameMultiArray>(
        "motor_command", rclcpp::QoS(1).best_effort(),
        [this](const MotorFrameMultiArray::SharedPtr msg) {
            motor_command_callback(msg);
        }
    );

    motor_state_publisher_ = this->create_publisher<MotorFrameMultiArray>(
        "motor_state", rclcpp::QoS(10).best_effort()
    );

    motor_state_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1),
        [this]() {
            motor_state_callback();
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
        motor_manager_->request_stop();
    }
    if (manager_run_thread_.joinable()) {
        manager_run_thread_.join();
    }
}

void MotorManagerNode::motor_command_callback(const MotorFrameMultiArray::SharedPtr msg)
{
    const size_t n_msg = msg->data.size();
    const uint8_t size = static_cast<uint8_t>(std::min(
        n_msg,
        static_cast<size_t>(motor_manager::MAX_CONTROLLER_SIZE)));
    motor_interface::motor_frame_t motor_frame[motor_manager::MAX_CONTROLLER_SIZE] = {};

    for (uint8_t i = 0; i < size; i++) {
        motor_frame[i].number_of_target_interfaces = std::min(
            msg->data[i].number_of_target_interfaces,
            motor_interface::MAX_INTERFACE_SIZE);
        const uint8_t n_if = motor_frame[i].number_of_target_interfaces;
        const size_t vec_n = msg->data[i].target_interface_id.size();
        const uint8_t copy_n = static_cast<uint8_t>(std::min(
            static_cast<size_t>(n_if),
            vec_n));
        for (uint8_t j = 0; j < copy_n; j++) {
            motor_frame[i].target_interface_id[j] = msg->data[i].target_interface_id[j];
        }
        motor_frame[i].controller_index = msg->data[i].controller_index;
        motor_frame[i].controlword = msg->data[i].controlword;
        motor_frame[i].statusword = msg->data[i].statusword;
        motor_frame[i].errorcode = msg->data[i].errorcode;
        motor_frame[i].position = msg->data[i].position;
        motor_frame[i].velocity = msg->data[i].velocity;
        motor_frame[i].torque = msg->data[i].torque;
    }

    motor_manager_->write(motor_frame, size);
}

void MotorManagerNode::motor_state_callback()
{
    const uint8_t n = motor_manager_->number_of_controllers();
    if (n == 0) {
        return;
    }

    motor_interface::motor_frame_t status[motor_manager::MAX_CONTROLLER_SIZE] = {};
    motor_manager_->read(status);

    MotorFrameMultiArray msg;
    msg.data.resize(n);
    for (uint8_t i = 0; i < n; ++i) {
        msg.data[i].number_of_target_interfaces = status[i].number_of_target_interfaces;
        const uint8_t ti_count = std::min(
            static_cast<uint8_t>(status[i].number_of_target_interfaces),
            motor_interface::MAX_INTERFACE_SIZE);
        msg.data[i].target_interface_id.resize(ti_count);
        for (uint8_t j = 0; j < ti_count; ++j) {
            msg.data[i].target_interface_id[j] = status[i].target_interface_id[j];
        }
        msg.data[i].controller_index = status[i].controller_index;
        msg.data[i].controlword = status[i].controlword;
        msg.data[i].statusword = status[i].statusword;
        msg.data[i].errorcode = status[i].errorcode;
        msg.data[i].position = status[i].position;
        msg.data[i].velocity = status[i].velocity;
        msg.data[i].torque = status[i].torque;
    }

    motor_state_publisher_->publish(msg);
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotorManagerNode>());
    rclcpp::shutdown();
    return 0;
}
