#pragma once
#include <memory>
#include <vector>

#include <rclcpp/node.hpp>
#include <rclcpp/service.hpp>
#include <rclcpp/subscription.hpp>

#include <tmx_cpp/modules/PCA9685.hpp>

#include <mirte_telemetrix_cpp/modules/base_module.hpp>
#include <mirte_telemetrix_cpp/modules/pca/pca_motor.hpp>
#include <mirte_telemetrix_cpp/modules/pca/pca_servo.hpp>
#include <mirte_telemetrix_cpp/node_data.hpp>

#include <mirte_telemetrix_cpp/parsers/modules/pca_data.hpp>

#include <mirte_msgs/msg/set_speed_named_array.hpp>
#include <mirte_msgs/srv/set_speed_multiple.hpp>

class PCA_Module : public Mirte_module {
public:
  PCA_Module(NodeData node_data, PCAData pca_data,
             std::shared_ptr<tmx_cpp::Modules> modules);
  std::shared_ptr<tmx_cpp::PCA9685_module> pca9685;

  std::vector<std::shared_ptr<PCAMotor>> motors;
  std::vector<std::shared_ptr<PCAServo>> servos;

  static std::vector<std::shared_ptr<PCA_Module>>
  get_pca_modules(NodeData node_data, std::shared_ptr<Parser> parser,
                  std::shared_ptr<tmx_cpp::Modules> modules);
  ~PCA_Module(){};

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
