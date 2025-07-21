#include <rcl/service_introspection.h>
#include <rclcpp/parameter_event_handler.hpp>
#include <rclcpp/qos.hpp>

#include <mirte_telemetrix_cpp/device_service_introspection.hpp>

DeviceServiceIntrospection::DeviceServiceIntrospection(
    rclcpp::Node::SharedPtr node,
    std::shared_ptr<rclcpp::ParameterEventHandler> param_event_handler,
    std::string device_param_key)
    : device_param_key(device_param_key), node_(node),
      service_introspection_calls_({}), introspect_enable_(false) {
  auto parameter_name = !this->device_param_key.empty()
                            ? this->device_param_key + ".service_introspection"
                            : "service_introspection";

  auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
  descriptor.description =
      "If the services from this device have service introspection enabled.";
  descriptor.read_only = !(bool)param_event_handler;

  this->introspect_enable_ =
      this->node_->declare_parameter(parameter_name, false);

  auto cb = [this](const rclcpp::Parameter &p) {
    auto enable_introspect = p.as_bool();

    if (this->introspect_enable_ == enable_introspect) {
      return;
    }

    auto clock = this->node_->get_clock();
    auto qos = rclcpp::SystemDefaultsQoS();
    auto introspection_mode = enable_introspect
                                  ? RCL_SERVICE_INTROSPECTION_CONTENTS
                                  : RCL_SERVICE_INTROSPECTION_OFF;
    for (auto &service_callback : this->service_introspection_calls_) {
      service_callback(clock, qos, introspection_mode);
    }
    this->introspect_enable_ = enable_introspect;
  };

  if (param_event_handler) {
    this->introspect_param_callback_ =
        param_event_handler->add_parameter_callback(parameter_name, cb);
  }
}
