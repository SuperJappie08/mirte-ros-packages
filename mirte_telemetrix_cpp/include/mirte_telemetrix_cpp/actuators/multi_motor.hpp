#pragma once

#include <mirte_telemetrix_cpp/device.hpp>
#include <mirte_telemetrix_cpp/node_data.hpp>

#include <mirte_telemetrix_cpp/actuators/motor.hpp>

#include <mirte_telemetrix_cpp/parsers/actuators/multi_motor_data.hpp>

#include <mirte_msgs/msg/set_speed_named_array.hpp>
#include <mirte_msgs/srv/set_speed_multiple.hpp>

class MultiMotor : public TelemetrixDevice {
public:
  MultiMotor(NodeData node_data, MultiMotorData multi_motor_data,
             std::vector<std::shared_ptr<Motor>> motors);

  std::vector<std::shared_ptr<Motor>> motors;

  static std::vector<std::shared_ptr<MultiMotor>>
  get_multi_motors(NodeData node_data, std::shared_ptr<Parser> parser,
                   std::vector<std::shared_ptr<Motor>> motors);

  using MultiSpeedType = mirte_msgs::msg::SetSpeedNamedArray::_speeds_type;

private:
  // Subscriber: motor/NAME/multi_speed
  rclcpp::Subscription<mirte_msgs::msg::SetSpeedNamedArray>::SharedPtr
      multi_speed_subscriber;
  // Service: motor/NAME/set_multiple_speeds
  rclcpp::Service<mirte_msgs::srv::SetSpeedMultiple>::SharedPtr motor_service;

  /// Set multiple speeds:
  ///
  /// Returns:
  ///  True on succes
  ///  False on complete failure (No Speeds where set)
  bool set_multi_speed(const MultiSpeedType &speeds);

  void multi_speed_subscription_callback(
      const mirte_msgs::msg::SetSpeedNamedArray &msg);

  void set_multi_speed_service_callback(
      const mirte_msgs::srv::SetSpeedMultiple::Request::ConstSharedPtr req,
      mirte_msgs::srv::SetSpeedMultiple::Response::SharedPtr res);
};
