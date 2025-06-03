#include <algorithm>
#include <functional>

#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>

#include <mirte_telemetrix_cpp/modules/pca_module.hpp>

using namespace std::placeholders; // for _1, _2, _3...

std::vector<std::shared_ptr<PCA_Module>>
PCA_Module::get_pca_modules(NodeData node_data, std::shared_ptr<Parser> parser,
                            std::shared_ptr<tmx_cpp::Modules> modules) {
  std::vector<std::shared_ptr<PCA_Module>> pca_modules;
  auto pca_data = parse_all_modules<PCAData>(parser, node_data.board);
  for (auto pca : pca_data) {
    auto pca_module = std::make_shared<PCA_Module>(node_data, pca, modules);
    pca_modules.push_back(pca_module);
  }
  return pca_modules;
}

// NOTE: Each motor has its own callback group since they inherit from the
// actuator::Motor
PCA_Module::PCA_Module(NodeData node_data, PCAData pca_data,
                       std::shared_ptr<tmx_cpp::Modules> modules)
    : Mirte_module(node_data, {pca_data.scl, pca_data.sda},
                   (ModuleData)pca_data) {
  this->device_timer->cancel();
  tmx->setI2CPins(pca_data.sda, pca_data.scl, pca_data.port);

  this->pca9685 = std::make_shared<tmx_cpp::PCA9685_module>(
      pca_data.port, pca_data.addr, pca_data.frequency);

  for (auto motor : pca_data.motors) {
    this->motors.push_back(
        std::make_shared<PCAMotor>(node_data, motor, pca9685));
  }
  for (auto servo : pca_data.servos) {
    this->servos.push_back(
        std::make_shared<PCAServo>(node_data, servo, pca9685));
  }

  rclcpp::SubscriptionOptions options;
  options.callback_group = this->callback_group;
  multi_speed_subscriber =
      nh->create_subscription<mirte_msgs::msg::SetSpeedNamedArray>(
          "motor/" + this->name + "/multi_speed", rclcpp::SystemDefaultsQoS(),
          std::bind(&PCA_Module::multi_speed_subscription_callback, this, _1),
          options);

  motor_service = nh->create_service<mirte_msgs::srv::SetSpeedMultiple>(
      "motor/" + this->name + "/set_multiple_speeds",
      std::bind(&PCA_Module::set_multi_speed_service_callback, this, _1, _2),
      rclcpp::ServicesQoS(), this->callback_group);

  modules->add_mod(pca9685);
}

void PCA_Module::multi_speed_subscription_callback(
    const mirte_msgs::msg::SetSpeedNamedArray &msg) {
  if (!set_multi_speed(msg.speeds)) {
    RCLCPP_ERROR(logger,
                 "An error occurred when trying to set multiple speeds!");
  }
}

void PCA_Module::set_multi_speed_service_callback(
    const mirte_msgs::srv::SetSpeedMultiple::Request::ConstSharedPtr req,
    mirte_msgs::srv::SetSpeedMultiple::Response::SharedPtr res) {
  res->success = set_multi_speed(req->speeds);
  if (!res->success) {
    RCLCPP_ERROR(logger,
                 "An error occurred when trying to set multiple speeds!");
  }
}

bool PCA_Module::set_multi_speed(const MultiSpeedType &speeds) {
  if (speeds.empty()) {
    RCLCPP_WARN(
        logger,
        "Tried to set multiple motor speeds, but no speeds where provided.");
    return false;
  }

  std::vector<tmx_cpp::PCA9685_module::PWM_val> pwm_vals;

  for (auto speed : speeds) {
    auto motor_name = speed.name;
    if (auto motor = std::find_if(
            motors.begin(), motors.end(),
            [motor_name](auto motor) { return motor->name == motor_name; });
        motor != motors.end()) {
      auto motor_pwm_vals = (*motor)->get_multi_speed_pwm(speed.speed);
      pwm_vals.insert(pwm_vals.end(), motor_pwm_vals.begin(),
                      motor_pwm_vals.end());
    } else {
      RCLCPP_WARN(logger,
                  "PCA Motor '%s' could not be found. Ignored for multiple "
                  "motor speed request.",
                  motor_name.c_str());
    }
  }

  if (pwm_vals.empty()) {
    RCLCPP_ERROR(
        logger,
        "None of the named motor speeds where found inn the PCA Motors.");
    return false;
  }

  return pca9685->set_multiple_pwm(pwm_vals);
}
