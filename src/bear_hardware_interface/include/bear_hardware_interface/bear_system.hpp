#ifndef BEAR_HARDWARE_INTERFACE__BEAR_SYSTEM_HPP_
#define BEAR_HARDWARE_INTERFACE__BEAR_SYSTEM_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp/rclcpp.hpp"

#include "bear_sdk.h"
#include "bear_macro.h"

namespace bear_hardware_interface
{

enum class control_mode_t {
  POSITION,
  VELOCITY,
  EFFORT,
  NONE
};

class BearSystemHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(BearSystemHardware)

  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;


  hardware_interface::return_type prepare_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;

  hardware_interface::return_type perform_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;

private:
  std::vector<double> hw_commands_positions_;
  std::vector<double> hw_commands_velocities_;
  std::vector<double> hw_commands_efforts_;
  
  std::vector<double> hw_states_positions_;
  std::vector<double> hw_states_velocities_;
  std::vector<double> hw_states_efforts_;

  std::vector<control_mode_t> control_modes_;

  std::shared_ptr<bear::BEAR> bear_instance_;
  std::string port_name_ = "/dev/ttyUSB0";
  int baudrate_ = 8000000;
  std::vector<int> motor_ids_;
  
  const double Kt_ = 0.35; // par-current constant for the Bear motors, in Nm/A
};
}  // namespace bear_hardware_interface

#endif // BEAR_HARDWARE_INTERFACE__BEAR_SYSTEM_HPP_