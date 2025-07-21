#pragma once
#include <memory>

#include <rclcpp/node.hpp>
#include <rclcpp/parameter_event_handler.hpp>

#include <tmx_cpp/tmx.hpp>

class Mirte_Board;
struct NodeData {
  std::shared_ptr<rclcpp::Node> nh;
  std::shared_ptr<tmx_cpp::TMX> tmx;
  std::shared_ptr<Mirte_Board> board;
  std::shared_ptr<rclcpp::ParameterEventHandler> param_event_handler;
};
