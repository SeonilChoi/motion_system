#include <algorithm>
#include <chrono>
#include <cmath>
#include <cctype>
#include <cstdint>
#include <functional>
#include <map>
#include <mutex>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include <yaml-cpp/yaml.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <midi_msgs/msg/midi.hpp>
#include <motion_control_msgs/msg/motor_status.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int8_multi_array.hpp>

namespace
{

constexpr std::size_t kNumMidiChannels = 8;
constexpr int32_t kFaderValueMax = 16383;
constexpr int32_t kDialValueMax = 127;
constexpr uint16_t kCwNewSetPointZeroerr = 0x103F;
constexpr uint16_t kCwNewSetPointMinas = 0x003F;
constexpr uint16_t kCwSocketcanSetPoint = 0x0001;
constexpr uint16_t kCwDynamixelEffortEnable = 1;

constexpr double kFaderEpsilon = 0.5;
constexpr const char * kConfigPackage = "motion_control_bridge";
constexpr const char * kConfigRelativePath = "/config/example_socketcan_cubemars.yaml";
constexpr const char * kMidiTopic = "/xtouch/midi";
constexpr const char * kCommandTopic = "motion_control/motor_command";
constexpr auto kPublishPeriod = std::chrono::milliseconds(5);
constexpr double kMaxSmoothingTimeMs = 3000.0;
constexpr double kSmoothingCurvePower = 2.0;

std::string default_config_file()
{
  return ament_index_cpp::get_package_share_directory(kConfigPackage) +
         kConfigRelativePath;
}

std::string to_lower(std::string s)
{
  std::transform(
    s.begin(), s.end(), s.begin(),
    [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return s;
}

template<typename T>
T required_as(const YAML::Node & node, const char * key, const std::string & context)
{
  if (!node[key]) {
    throw std::runtime_error("Missing '" + std::string(key) + "' in " + context);
  }
  return node[key].as<T>();
}

template<typename Vector>
bool bool_at(const Vector & values, std::size_t index)
{
  return index < values.size() && static_cast<bool>(values[index]);
}

template<typename Vector>
int32_t int_at(
  const Vector & values,
  std::size_t index,
  int32_t fallback = 0)
{
  if (index >= values.size()) {
    return fallback;
  }
  return values[index];
}

}  // namespace

class MotionControlMidiNode : public rclcpp::Node
{
public:
  using MidiMsg = midi_msgs::msg::Midi;
  using MotorStatus = motion_control_msgs::msg::MotorStatus;

  explicit MotionControlMidiNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : rclcpp::Node("motion_control_midi_node", options)
  {
    const std::string config_file = default_config_file();
    publish_period_ = kPublishPeriod;
    max_smoothing_time_ms_ = kMaxSmoothingTimeMs;
    smoothing_curve_power_ = kSmoothingCurvePower;

    motor_infos_ = load_motor_infos(config_file);
    if (motor_infos_.empty()) {
      throw std::runtime_error("No motor controllers were found in " + config_file);
    }
    motor_states_.resize(motor_infos_.size());

    motor_command_pub_ = create_publisher<MotorStatus>(
      kCommandTopic, rclcpp::QoS(1).best_effort());

    midi_sub_ = create_subscription<MidiMsg>(
      kMidiTopic, rclcpp::QoS(1).best_effort(),
      std::bind(&MotionControlMidiNode::midi_callback, this, std::placeholders::_1));

    command_tick_ = create_wall_timer(
      publish_period_, std::bind(&MotionControlMidiNode::publish_motor_command, this));

    RCLCPP_INFO(
      get_logger(),
      "motion_control_midi_node ready. Loaded %zu controller(s) from '%s'; "
      "subscribing '%s', publishing '%s'.",
      motor_infos_.size(), config_file.c_str(), kMidiTopic, kCommandTopic);
  }

private:
  struct DriverInfo
  {
    uint8_t id{};
    double lower{};
    double upper{};
    double speed{};
    double rated_effort{};
    std::string type;
  };

  struct MotorInfo
  {
    uint8_t controller_index{};
    int8_t profile_mode{};
    DriverInfo driver;
  };

  struct MotorCommandState
  {
    bool active{};
    bool initialized{};
    int32_t target_fader{};
    double current_fader{};
    int32_t dial{};
  };

  struct PendingTarget
  {
    std::size_t motor_index{};
    int32_t fader{};
    int32_t dial{};
  };

  void midi_callback(const MidiMsg::SharedPtr msg)
  {
    std::vector<PendingTarget> targets;
    targets.reserve(kNumMidiChannels);

    for (std::size_t ch = 0; ch < kNumMidiChannels; ++ch) {
      if (!bool_at(msg->btn3, ch)) {
        continue;
      }

      const std::size_t motor_index = ch + bank_offset(*msg, ch);
      if (motor_index >= motor_infos_.size()) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Ignoring MIDI ch%zu -> motor %zu; config has %zu controller(s).",
          ch, motor_index, motor_infos_.size());
        continue;
      }

      PendingTarget target;
      target.motor_index = motor_index;
      target.fader = std::clamp(
        int_at(msg->channel, ch), int32_t{0}, kFaderValueMax);
      target.dial = std::clamp(
        int_at(msg->dial, ch), int32_t{0}, kDialValueMax);
      targets.push_back(target);
    }

    std::lock_guard<std::mutex> lk(state_mutex_);
    for (auto & state : motor_states_) {
      state.active = false;
    }

    for (const auto & target : targets) {
      MotorCommandState & state = motor_states_[target.motor_index];
      if (!state.initialized) {
        state.current_fader = target.fader;
        state.initialized = true;
      }
      state.active = true;
      state.target_fader = target.fader;
      state.dial = target.dial;
    }
  }

  void publish_motor_command()
  {
    MotorStatus msg = make_empty_motor_status();
    bool has_target = false;

    {
      std::lock_guard<std::mutex> lk(state_mutex_);
      for (std::size_t motor_index = 0; motor_index < motor_states_.size(); ++motor_index) {
        MotorCommandState & state = motor_states_[motor_index];
        if (!state.active) {
          continue;
        }

        const double alpha = smoothing_alpha(state.dial);
        const double error =
          static_cast<double>(state.target_fader) - state.current_fader;
        if (std::abs(error) <= kFaderEpsilon || alpha >= 1.0) {
          state.current_fader = state.target_fader;
        } else {
          state.current_fader += error * alpha;
        }

        const int32_t fader_value = std::clamp<int32_t>(
          static_cast<int32_t>(std::lround(state.current_fader)),
          0, kFaderValueMax);
        has_target =
          fill_motor_command_target(msg, motor_index, fader_value) || has_target;
      }
    }

    if (has_target) {
      motor_command_pub_->publish(msg);
    }
  }

  std::size_t bank_offset(const MidiMsg & msg, std::size_t channel) const
  {
    std::size_t offset = 0;
    if (bool_at(msg.btn1, channel)) {
      offset += 8;
    }
    if (bool_at(msg.btn2, channel)) {
      offset += 16;
    }
    return offset;
  }

  double smoothing_alpha(int32_t dial) const
  {
    if (dial <= 0 || max_smoothing_time_ms_ <= 0.0) {
      return 1.0;
    }

    const double normalized =
      static_cast<double>(std::clamp(dial, int32_t{0}, kDialValueMax)) /
      static_cast<double>(kDialValueMax);
    const double curved = std::pow(normalized, smoothing_curve_power_);
    const double tau_ms = curved * max_smoothing_time_ms_;
    if (tau_ms <= 0.0) {
      return 1.0;
    }

    const double dt_ms = static_cast<double>(publish_period_.count());
    return std::clamp(1.0 - std::exp(-dt_ms / tau_ms), 0.001, 1.0);
  }

  std::vector<MotorInfo> load_motor_infos(const std::string & config_file) const
  {
    YAML::Node root = YAML::LoadFile(config_file);
    if (!root) {
      throw std::runtime_error("Failed to load motor config: " + config_file);
    }

    const YAML::Node drivers_node = root["drivers"];
    if (!drivers_node || !drivers_node.IsSequence()) {
      throw std::runtime_error("Invalid or missing 'drivers' in " + config_file);
    }

    std::unordered_map<int, DriverInfo> drivers;
    for (const auto & driver_node : drivers_node) {
      const int id = required_as<int>(driver_node, "id", "drivers[]");
      DriverInfo info;
      info.id = static_cast<uint8_t>(id);
      info.lower = required_as<double>(driver_node, "lower", "drivers[]");
      info.upper = required_as<double>(driver_node, "upper", "drivers[]");
      info.speed = required_as<double>(driver_node, "speed", "drivers[]");
      info.rated_effort =
        required_as<double>(driver_node, "rated_effort", "drivers[]");
      info.type = to_lower(required_as<std::string>(driver_node, "type", "drivers[]"));
      drivers.emplace(id, info);
    }

    const YAML::Node masters_node = root["masters"];
    if (!masters_node || !masters_node.IsSequence()) {
      throw std::runtime_error("Invalid or missing 'masters' in " + config_file);
    }

    std::map<int, MotorInfo> motors_by_index;
    for (const auto & master_node : masters_node) {
      const YAML::Node slaves_node = master_node["slaves"];
      if (!slaves_node || !slaves_node.IsSequence()) {
        throw std::runtime_error("Invalid or missing 'slaves' in masters[]");
      }

      for (const auto & slave_node : slaves_node) {
        const int controller_index =
          required_as<int>(slave_node, "controller_index", "slaves[]");
        const int driver_id = required_as<int>(slave_node, "driver_id", "slaves[]");
        const int profile_mode =
          required_as<int>(slave_node, "profile_mode", "slaves[]");

        if (controller_index < 0 || controller_index > 255) {
          throw std::runtime_error("controller_index is out of uint8 range.");
        }

        const auto driver_iter = drivers.find(driver_id);
        if (driver_iter == drivers.end()) {
          throw std::runtime_error(
            "No driver entry for driver_id " + std::to_string(driver_id));
        }

        MotorInfo info;
        info.controller_index = static_cast<uint8_t>(controller_index);
        info.profile_mode = static_cast<int8_t>(profile_mode);
        info.driver = driver_iter->second;

        const auto insert_result = motors_by_index.emplace(controller_index, info);
        if (!insert_result.second) {
          throw std::runtime_error(
            "Duplicate controller_index " + std::to_string(controller_index));
        }
      }
    }

    std::vector<MotorInfo> result;
    result.reserve(motors_by_index.size());
    int expected_index = 0;
    for (const auto & [controller_index, info] : motors_by_index) {
      if (controller_index != expected_index) {
        throw std::runtime_error(
          "controller_index values must be dense from 0 for motor_manager.");
      }
      result.push_back(info);
      ++expected_index;
    }

    return result;
  }

  MotorStatus make_empty_motor_status() const
  {
    MotorStatus msg;
    const std::size_t n = motor_infos_.size();

    msg.number_of_target_interfaces.assign(n, 0);
    msg.target_interface_id.resize(n);
    msg.controller_index.resize(n);
    msg.controlword.assign(n, 0);
    msg.statusword.assign(n, 0);
    msg.errorcode.assign(n, 0);
    msg.position.assign(n, 0.0);
    msg.velocity.assign(n, 0.0);
    msg.effort.assign(n, 0.0);

    for (std::size_t i = 0; i < n; ++i) {
      msg.controller_index[i] = motor_infos_[i].controller_index;
    }

    return msg;
  }

  bool fill_motor_command_target(
    MotorStatus & msg, std::size_t motor_index, int32_t fader_value)
  {
    if (motor_index >= motor_infos_.size()) {
      return false;
    }

    const MotorInfo & motor = motor_infos_[motor_index];
    const std::size_t idx = motor.controller_index;

    if (idx >= msg.controller_index.size()) {
      RCLCPP_WARN(
        get_logger(), "Controller index %zu is outside MotorStatus array.", idx);
      return false;
    }

    switch (motor.profile_mode) {
      case 0: {
        if (to_lower(motor.driver.type) == "dynamixel") {
          msg.number_of_target_interfaces[idx] = 1;
          msg.target_interface_id[idx].data = std::vector<int8_t>{1};
        } else {
          msg.number_of_target_interfaces[idx] = 2;
          msg.target_interface_id[idx].data = std::vector<int8_t>{0, 1};
          msg.controlword[idx] = controlword_for_driver(motor.driver.type);
        }
        msg.position[idx] = scale_fader(
          fader_value, motor.driver.lower, motor.driver.upper);
        break;
      }
      case 1: {
        msg.number_of_target_interfaces[idx] = 1;
        msg.target_interface_id[idx].data = std::vector<int8_t>{2};
        const double degrees_per_second = scale_fader(
          fader_value, -motor.driver.speed, motor.driver.speed);
        msg.velocity[idx] = degrees_per_second;
        break;
      }
      case 2: {
        msg.number_of_target_interfaces[idx] = 1;
        msg.target_interface_id[idx].data = std::vector<int8_t>{3};
        msg.effort[idx] = scale_fader(
          fader_value, -motor.driver.rated_effort, motor.driver.rated_effort);
        break;
      }
      default:
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Unsupported profile_mode %d for controller %u.",
          motor.profile_mode, motor.controller_index);
        return false;
    }

    return true;
  }

  double scale_fader(int32_t fader_value, double lower, double upper) const
  {
    const double normalized =
      static_cast<double>(std::clamp<int32_t>(fader_value, 0, kFaderValueMax)) /
      static_cast<double>(kFaderValueMax);
    return lower + (upper - lower) * normalized;
  }

  uint16_t controlword_for_driver(const std::string & driver_type) const
  {
    const std::string type = to_lower(driver_type);
    if (type == "zeroerr") {
      return kCwNewSetPointZeroerr;
    }
    if (type == "minas") {
      return kCwNewSetPointMinas;
    }
    if (type == "cubemars") {
      return kCwSocketcanSetPoint;
    }
    if (type == "dynamixel") {
      return kCwDynamixelEffortEnable;
    }

    RCLCPP_WARN(
      get_logger(), "Unknown driver type '%s'; using zeroerr set-point controlword.",
      driver_type.c_str());
    return kCwNewSetPointZeroerr;
  }

  rclcpp::Subscription<MidiMsg>::SharedPtr midi_sub_;
  rclcpp::Publisher<MotorStatus>::SharedPtr motor_command_pub_;
  rclcpp::TimerBase::SharedPtr command_tick_;

  std::vector<MotorInfo> motor_infos_;
  std::vector<MotorCommandState> motor_states_;
  std::mutex state_mutex_;
  std::chrono::milliseconds publish_period_{5};
  double max_smoothing_time_ms_{3000.0};
  double smoothing_curve_power_{2.0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<MotionControlMidiNode>());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(
      rclcpp::get_logger("motion_control_midi_node"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
