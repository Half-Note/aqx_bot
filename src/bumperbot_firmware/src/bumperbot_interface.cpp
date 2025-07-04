#include "bumperbot_firmware/bumperbot_interface.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <cstring>   // memset
#include <sstream>
#include <iostream>
#include <iomanip>

namespace bumperbot_firmware
{

BumperbotInterface::BumperbotInterface()
: udp_socket_(-1)
{
}

BumperbotInterface::~BumperbotInterface()
{
  if (udp_socket_ != -1)
  {
    close(udp_socket_);
    udp_socket_ = -1;
  }
}

CallbackReturn BumperbotInterface::on_init(const hardware_interface::HardwareInfo &hardware_info)
{
  CallbackReturn result = hardware_interface::SystemInterface::on_init(hardware_info);
  if (result != CallbackReturn::SUCCESS)
  {
    return result;
  }

  // Read UDP parameters from URDF/launch
  try
  {
    ip_address_ = hardware_info.hardware_parameters.at("ip_address");
    udp_port_ = std::stoi(hardware_info.hardware_parameters.at("udp_port"));
  }
  catch (const std::exception &e)
  {
    RCLCPP_FATAL(rclcpp::get_logger("BumperbotInterface"), "Failed to get UDP parameters: %s", e.what());
    return CallbackReturn::FAILURE;
  }

  velocity_commands_.resize(hardware_info.joints.size(), 0.0);
  position_states_.resize(hardware_info.joints.size(), 0.0);
  velocity_states_.resize(hardware_info.joints.size(), 0.0);

  last_run_ = rclcpp::Clock().now();

  return CallbackReturn::SUCCESS;
}

CallbackReturn BumperbotInterface::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("BumperbotInterface"), "Starting robot hardware (UDP) ...");

  // Create UDP socket
  udp_socket_ = socket(AF_INET, SOCK_DGRAM, 0);
  if (udp_socket_ < 0)
  {
    RCLCPP_FATAL(rclcpp::get_logger("BumperbotInterface"), "Failed to create UDP socket");
    return CallbackReturn::FAILURE;
  }

  memset(&robot_addr_, 0, sizeof(robot_addr_));
  robot_addr_.sin_family = AF_INET;
  robot_addr_.sin_port = htons(udp_port_);
  if (inet_pton(AF_INET, ip_address_.c_str(), &robot_addr_.sin_addr) <= 0)
  {
    RCLCPP_FATAL(rclcpp::get_logger("BumperbotInterface"), "Invalid IP address: %s", ip_address_.c_str());
    close(udp_socket_);
    udp_socket_ = -1;
    return CallbackReturn::FAILURE;
  }

  RCLCPP_INFO(rclcpp::get_logger("BumperbotInterface"), "Hardware started, ready to take commands");
  return CallbackReturn::SUCCESS;
}

CallbackReturn BumperbotInterface::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("BumperbotInterface"), "Stopping robot hardware ...");

  if (udp_socket_ != -1)
  {
    close(udp_socket_);
    udp_socket_ = -1;
  }

  RCLCPP_INFO(rclcpp::get_logger("BumperbotInterface"), "Hardware stopped");
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> BumperbotInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (size_t i = 0; i < position_states_.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_states_[i]));

    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_states_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> BumperbotInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (size_t i = 0; i < velocity_commands_.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_commands_[i]));
  }

  return command_interfaces;
}

hardware_interface::return_type BumperbotInterface::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  // Receive UDP packet non-blocking
  char buffer[128];
  ssize_t len = recvfrom(udp_socket_, buffer, sizeof(buffer)-1, MSG_DONTWAIT, nullptr, nullptr);
  if (len > 0)
  {
    buffer[len] = '\0';
    // Parse incoming data to update position_states_ and velocity_states_
    // Expected format: r<p or n><value>,l<p or n><value>, ...
    std::stringstream ss(buffer);
    std::string token;
    double dt = (rclcpp::Clock().now() - last_run_).seconds();

    while(std::getline(ss, token, ','))
    {
      if (token.size() < 3) continue;
      char wheel = token[0]; // 'r' or 'l'
      char sign = token[1];  // 'p' or 'n'
      double val = std::stod(token.substr(2));

      double signed_val = (sign == 'p') ? val : -val;

      if (wheel == 'r')
      {
        velocity_states_[0] = signed_val;
        position_states_[0] += signed_val * dt;
      }
      else if (wheel == 'l')
      {
        velocity_states_[1] = signed_val;
        position_states_[1] += signed_val * dt;
      }
    }
    last_run_ = rclcpp::Clock().now();
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type BumperbotInterface::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  std::stringstream message_stream;
  char right_wheel_sign = velocity_commands_[0] >= 0 ? 'p' : 'n';
  char left_wheel_sign = velocity_commands_[1] >= 0 ? 'p' : 'n';

  std::string compensate_zeros_right = (std::abs(velocity_commands_[0]) < 10.0) ? "0" : "";
  std::string compensate_zeros_left = (std::abs(velocity_commands_[1]) < 10.0) ? "0" : "";

  message_stream << std::fixed << std::setprecision(2)
                 << "r" << right_wheel_sign << compensate_zeros_right << std::abs(velocity_commands_[0])
                 << ",l" << left_wheel_sign << compensate_zeros_left << std::abs(velocity_commands_[1]) << ",";

  std::string message = message_stream.str();
  ssize_t sent_len = sendto(udp_socket_, message.c_str(), message.size(), 0,
                            (struct sockaddr *)&robot_addr_, sizeof(robot_addr_));

  if (sent_len < 0)
  {
    RCLCPP_ERROR(rclcpp::get_logger("BumperbotInterface"),
                 "Failed to send UDP message: %s", message.c_str());
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

}  // namespace bumperbot_firmware

PLUGINLIB_EXPORT_CLASS(bumperbot_firmware::BumperbotInterface, hardware_interface::SystemInterface)
