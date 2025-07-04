#include <algorithm>
#include <functional>
#include <vector>

#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>

#include <mirte_telemetrix_cpp/actuators/motor.hpp>
#include <mirte_telemetrix_cpp/actuators/multi_motor.hpp>

using namespace std::placeholders; // for _1, _2, _3...

std::vector<std::shared_ptr<MultiMotor>>
MultiMotor::get_multi_motors(NodeData node_data, std::shared_ptr<Parser> parser,
                             std::vector<std::shared_ptr<Motor>> motors) {
  std::vector<std::shared_ptr<MultiMotor>> multi_motors;
  auto multi_motor_datas = parse_all<MultiMotorData>(parser, node_data.board);
  // TODO(SuperJappie08): Implement Singleton behavior
  for (auto device_data : multi_motor_datas) {
    if (device_data.check()) {
      if (multi_motors.empty()) {
        multi_motors.push_back(
            std::make_shared<MultiMotor>(node_data, device_data, motors));
      } else {
        RCLCPP_ERROR(parser->logger,
                     "There can only be a single instance of a %s, using first "
                     "instance (%s.%s) and ignoring %s.%s",
                     MultiMotorData::get_device_class().c_str(),
                     MultiMotorData::get_device_class().c_str(),
                     multi_motors[0]->name.c_str(),
                     MultiMotorData::get_device_class().c_str(),
                     device_data.name.c_str());
      }
    }
  }
  return multi_motors;
}

MultiMotor::MultiMotor(NodeData node_data, MultiMotorData multi_motor_data,
                       std::vector<std::shared_ptr<Motor>> motors)
    : TelemetrixDevice(node_data, {}, (DeviceData)multi_motor_data,
                       rclcpp::CallbackGroupType::MutuallyExclusive),
      motors(motors) {
  rclcpp::SubscriptionOptions options;
  options.callback_group = this->callback_group;
  multi_speed_subscriber =
      nh->create_subscription<mirte_msgs::msg::SetSpeedNamedArray>(
          "motor/" + this->name + "/multi_speed", rclcpp::SystemDefaultsQoS(),
          std::bind(&MultiMotor::multi_speed_subscription_callback, this, _1),
          options);

  motor_service = nh->create_service<mirte_msgs::srv::SetSpeedMultiple>(
      "motor/" + this->name + "/set_multiple_speeds",
      std::bind(&MultiMotor::set_multi_speed_service_callback, this, _1, _2),
      rclcpp::ServicesQoS(), this->callback_group);

  this->device_timer->cancel();
}

void MultiMotor::multi_speed_subscription_callback(
    const mirte_msgs::msg::SetSpeedNamedArray &msg) {
  if (!set_multi_speed(msg.speeds)) {
    RCLCPP_ERROR(logger,
                 "An error occurred when trying to set multiple speeds!");
  }
}

void MultiMotor::set_multi_speed_service_callback(
    const mirte_msgs::srv::SetSpeedMultiple::Request::ConstSharedPtr req,
    mirte_msgs::srv::SetSpeedMultiple::Response::SharedPtr res) {
  res->success = set_multi_speed(req->speeds);
  if (!res->success) {
    RCLCPP_ERROR(logger,
                 "An error occurred when trying to set multiple speeds!");
  }
}

bool MultiMotor::set_multi_speed(const MultiSpeedType &speeds) {
  if (speeds.empty()) {
    RCLCPP_WARN(
        logger,
        "Tried to set multiple motor speeds, but no speeds where provided.");
    return false;
  }

  for (auto speed : speeds) {
    auto motor_name = speed.name;
    if (auto motor = std::find_if(
            motors.begin(), motors.end(),
            [motor_name](auto motor) { return motor->name == motor_name; });
        motor != motors.end()) {
      /* TODO(SuperJappie08): Consider binding functions and then calling them
         one after another. This would reduce the delay between commands.*/
      // TODO(SuperJappie08): Add TMX multi PWM Command and utilize here.
      (*motor)->set_speed(speed.speed);
    } else {
      RCLCPP_WARN(logger,
                  "Motor '%s' could not be found. Ignored for multiple "
                  "motor speed request.",
                  motor_name.c_str());
    }
  }

  // TODO(SuperJappie08): Add No PWMS found warning. (Like PCA)
  return true;
}
