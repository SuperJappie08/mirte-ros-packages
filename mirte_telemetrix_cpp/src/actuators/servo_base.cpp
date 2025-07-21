#include <algorithm>
#include <functional>
#include <numbers>
#include <optional>

#include <rclcpp/callback_group.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>

#include <tmx_cpp/tmx.hpp>

#include <mirte_telemetrix_cpp/actuators/servo_base.hpp>

#include <mirte_msgs/srv/set_servo_angle.hpp>

/* The Servo callback group can be reentrant (Parrallel), since the second
 * callback does not influence the hardware. */
ServoBase::ServoBase(NodeData node_data, std::vector<pin_t> pins,
                     ServoData servo_data,
                     std::optional<std::string> device_key,
                     rclcpp::CallbackGroupType callback_group_type)
    : TelemetrixDevice(node_data, pins, (DeviceData)servo_data,
                       callback_group_type),
      data(servo_data) {
  using namespace std::placeholders;

  auto device_key_value =
      device_key.value_or(get_device_key<ServoData>(&servo_data));
  this->srv_manager = std::make_shared<DeviceServiceIntrospection>(
      node_data.nh, node_data.param_event_handler, device_key_value);

  this->set_angle_service =
      srv_manager->create_service<mirte_msgs::srv::SetServoAngle>(
          "servo/" + name + "/set_angle",
          std::bind(&ServoBase::set_angle_service_callback, this, _1, _2),
          rclcpp::ServicesQoS(), this->callback_group);

  this->get_range_service =
      srv_manager->create_service<mirte_msgs::srv::GetServoRange>(
          "servo/" + name + "/get_range",
          std::bind(&ServoBase::get_range_service_callback, this, _1, _2),
          rclcpp::ServicesQoS(), this->callback_group);

  this->device_timer->cancel();
}

void ServoBase::set_angle_service_callback(
    mirte_msgs::srv::SetServoAngle::Request::ConstSharedPtr req,
    mirte_msgs::srv::SetServoAngle::Response::SharedPtr res) {
  float angle = req->angle;
  bool is_degrees = req->degrees;

  if (is_degrees == mirte_msgs::srv::SetServoAngle::Request::RADIANS) {
    angle = angle * (180.0 / std::numbers::pi);
  }

  if (angle > data.max_angle || angle < data.min_angle) {
    RCLCPP_WARN(logger,
                "The provided angle is out of range. Angle %.3f degrees was "
                "requested, but range is [%.3f, "
                "%.3f]",
                angle, data.min_angle, data.max_angle);
    res->status = false;
    return;
  }

  float fraction = (angle - data.min_angle) / (data.max_angle - data.min_angle);
  uint16_t duty_cycle =
      (uint16_t)(fraction * (data.max_pulse - data.min_pulse)) + data.min_pulse;

  res->status = set_angle_us(duty_cycle);
}

void ServoBase::get_range_service_callback(
    const mirte_msgs::srv::GetServoRange::Request::ConstSharedPtr req,
    mirte_msgs::srv::GetServoRange::Response::SharedPtr res) {
  res->max = data.max_angle;
  res->min = data.min_angle;
}
