#pragma once

#include <mirte_telemetrix_cpp/parsers/device_data.hpp>

class MultiMotorData : public DeviceData {
public:
  bool check() override;

  MultiMotorData(std::shared_ptr<Parser> parser,
                 std::shared_ptr<Mirte_Board> board, std::string name,
                 std::map<std::string, rclcpp::ParameterValue> parameters,
                 std::set<std::string> &unused_keys);
  ~MultiMotorData(){};
  static std::string get_device_class() { return "multimotor"; }
};
