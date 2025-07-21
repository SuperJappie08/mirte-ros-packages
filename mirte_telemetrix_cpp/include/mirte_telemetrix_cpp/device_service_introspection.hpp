#pragma once

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <rcl/service_introspection.h>
#include <rclcpp/node.hpp>
#include <rclcpp/parameter_event_handler.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/service.hpp>

class DeviceServiceIntrospection {
public:
  DeviceServiceIntrospection(
      rclcpp::Node::SharedPtr node,
      std::shared_ptr<rclcpp::ParameterEventHandler> param_event_handler,
      std::string device_param_key);

  std::string device_param_key;

  template <typename ServiceT, typename CallbackT>
  typename rclcpp::Service<ServiceT>::SharedPtr
  create_service(const std::string &service_name, CallbackT &&callback,
                 const rclcpp::QoS &qos = rclcpp::ServicesQoS(),
                 rclcpp::CallbackGroup::SharedPtr group = nullptr) {
    using namespace std::placeholders;
    auto service = this->node_->create_service<ServiceT>(service_name, callback,
                                                         qos, group);

    this->service_introspection_calls_.push_back(
        std::bind(&rclcpp::Service<ServiceT>::configure_introspection, service,
                  _1, _2, _3));

    if (this->introspect_enable_) {
      service->configure_introspection(this->node_->get_clock(),
                                       rclcpp::ParameterEventsQoS(),
                                       RCL_SERVICE_INTROSPECTION_CONTENTS);
    }

    return service;
  }

private:
  rclcpp::Node::SharedPtr node_;
  std::vector<std::function<void(rclcpp::Clock::SharedPtr, rclcpp::QoS &,
                                 rcl_service_introspection_state_t)>>
      service_introspection_calls_;

  bool introspect_enable_;

  rclcpp::ParameterCallbackHandle::SharedPtr introspect_param_callback_ =
      nullptr;
};
