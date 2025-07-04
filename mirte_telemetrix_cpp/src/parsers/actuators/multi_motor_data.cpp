#include <mirte_telemetrix_cpp/parsers/actuators/multi_motor_data.hpp>

MultiMotorData::MultiMotorData(
    std::shared_ptr<Parser> parser, std::shared_ptr<Mirte_Board> board,
    std::string name, std::map<std::string, rclcpp::ParameterValue> parameters,
    std::set<std::string> &unused_keys)
    : DeviceData(parser, board, name, MultiMotorData::get_device_class(),
                 parameters, unused_keys) {
  auto logger = parser->logger;

  // NOTE(SuperJappie08): Disabled for now. as it is a weird requirement
  // if (name != MultiMotorData::get_device_class()) {
  //   RCLCPP_ERROR(logger, "Device %s.%s is of type %s and must be called %s",
  //                get_device_class().c_str(), name.c_str(),
  //                get_device_class().c_str(), get_device_class().c_str());
  // }

  // TODO: Currently no data
}

bool MultiMotorData::check() {
  bool device_ok = DeviceData::check();

  // NOTE(SuperJappie08): Disabled for now. as it is a weird requirement
  // // To ensure this is a singleton device it's name must be it's type
  // return (this->name == MultiMotorData::get_device_class()) && device_ok;
  return device_ok;
}
