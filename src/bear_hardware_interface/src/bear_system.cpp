#include "bear_hardware_interface/bear_system.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace bear_hardware_interface
{

hardware_interface::CallbackReturn BearSystemHardware::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  try {
      port_name_ = info_.hardware_parameters.at("port_name");
      baudrate_ = std::stoi(info_.hardware_parameters.at("baudrate"));
  } catch (const std::out_of_range &e) {
      RCLCPP_INFO(rclcpp::get_logger("BearSystemHardware"), "Usando configuración de puerto/baudrate por defecto.");
  }

  hw_states_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_states_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_states_efforts_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  
  hw_commands_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_efforts_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  control_modes_.resize(info_.joints.size(), control_mode_t::NONE);

  motor_ids_.resize(info_.joints.size());
  for (size_t i = 0; i < info_.joints.size(); i++) {
      if (info_.joints[i].parameters.find("motor_id") != info_.joints[i].parameters.end()) {
          motor_ids_[i] = std::stoi(info_.joints[i].parameters.at("motor_id"));
      } else {
          motor_ids_[i] = i + 1;
      }
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn BearSystemHardware::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("BearSystemHardware"), "Configurando BEAR en %s...", port_name_.c_str());
  try {
      bear_instance_ = std::make_shared<bear::BEAR>(port_name_.c_str(), baudrate_);

      for (int id : motor_ids_) {
          bear_instance_->SetLimitIMax(id, 3.0f);
          bear_instance_->SetPGainIq(id, 0.02f);
          bear_instance_->SetIGainIq(id, 0.02f);
          bear_instance_->SetDGainIq(id, 0.0f);
          bear_instance_->SetPGainId(id, 0.02f);
          bear_instance_->SetIGainId(id, 0.02f);
          bear_instance_->SetDGainId(id, 0.0f);
      }
  } catch (...) {
    RCLCPP_FATAL(rclcpp::get_logger("BearSystemHardware"), "Error al inicializar comunicación.");
    return hardware_interface::CallbackReturn::ERROR;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn BearSystemHardware::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("BearSystemHardware"), "Activando hardware...");
  
  for (size_t i = 0; i < motor_ids_.size(); i++) {
      int id = motor_ids_[i];

      bear_instance_->SetMode(id, 2);
      bear_instance_->SetPGainPosition(id, 5.0f);
      bear_instance_->SetIGainPosition(id, 0.0f);
      bear_instance_->SetDGainPosition(id, 0.2f);

      float current_pos = bear_instance_->GetPresentPosition(id);
      hw_states_positions_[i] = current_pos;
      hw_commands_positions_[i] = current_pos;
    
      control_modes_[i] = control_mode_t::POSITION;
      bear_instance_->SetTorqueEnable(id, 1);
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn BearSystemHardware::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  for (int id : motor_ids_) bear_instance_->SetTorqueEnable(id, 0);
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn BearSystemHardware::on_cleanup(const rclcpp_lifecycle::State & /*previous_state*/)
{
  bear_instance_.reset();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn BearSystemHardware::on_shutdown(const rclcpp_lifecycle::State & /*previous_state*/)
{
  for (int id : motor_ids_) bear_instance_->SetTorqueEnable(id, 0);
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type BearSystemHardware::prepare_command_mode_switch(
  const std::vector<std::string> & /*start_interfaces*/,
  const std::vector<std::string> & /*stop_interfaces*/)
{
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type BearSystemHardware::perform_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & /*stop_interfaces*/)
{
  for (const auto & key : start_interfaces) {
    for (size_t i = 0; i < info_.joints.size(); i++) {
      if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_POSITION) {
        bear_instance_->SetTorqueEnable(motor_ids_[i], 0);
        bear_instance_->SetMode(motor_ids_[i], 2); // Mode 2: Position
        bear_instance_->SetTorqueEnable(motor_ids_[i], 1);
        control_modes_[i] = control_mode_t::POSITION;
        RCLCPP_INFO(rclcpp::get_logger("BearSystemHardware"), "Motor %d -> MODO POSICIÓN", motor_ids_[i]);
      } 
      else if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_VELOCITY) {
        bear_instance_->SetTorqueEnable(motor_ids_[i], 0);
        bear_instance_->SetMode(motor_ids_[i], 1); // Mode 1: Velocity
        bear_instance_->SetTorqueEnable(motor_ids_[i], 1);
        control_modes_[i] = control_mode_t::VELOCITY;
        RCLCPP_INFO(rclcpp::get_logger("BearSystemHardware"), "Motor %d -> MODO VELOCIDAD", motor_ids_[i]);
      } 
      else if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_EFFORT) {
        bear_instance_->SetTorqueEnable(motor_ids_[i], 0);
        bear_instance_->SetMode(motor_ids_[i], 0); // Mode 0: Torque
        bear_instance_->SetTorqueEnable(motor_ids_[i], 1);
        control_modes_[i] = control_mode_t::EFFORT;
        RCLCPP_INFO(rclcpp::get_logger("BearSystemHardware"), "Motor %d -> MODO ESFUERZO", motor_ids_[i]);
      }
    }
  }
  return hardware_interface::return_type::OK;
}

// -----------------------------------------

hardware_interface::return_type BearSystemHardware::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  for (size_t i = 0; i < motor_ids_.size(); i++) {
      hw_states_positions_[i] = bear_instance_->GetPresentPosition(motor_ids_[i]);
      hw_states_velocities_[i] = bear_instance_->GetPresentVelocity(motor_ids_[i]);
      float iq = bear_instance_->GetPresentIq(motor_ids_[i]);
      hw_states_efforts_[i] = iq * Kt_; 
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type BearSystemHardware::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  for (size_t i = 0; i < motor_ids_.size(); i++) {
  
      if (control_modes_[i] == control_mode_t::POSITION && !std::isnan(hw_commands_positions_[i])) {
          bear_instance_->SetGoalPosition(motor_ids_[i], (float)hw_commands_positions_[i]);
      } 
      else if (control_modes_[i] == control_mode_t::VELOCITY && !std::isnan(hw_commands_velocities_[i])) {
          bear_instance_->SetGoalVelocity(motor_ids_[i], (float)hw_commands_velocities_[i]);
      } 
      else if (control_modes_[i] == control_mode_t::EFFORT && !std::isnan(hw_commands_efforts_[i])) {
       
          float target_iq = (float)(hw_commands_efforts_[i] / Kt_);
          bear_instance_->SetGoalIq(motor_ids_[i], target_iq);
      }
  }
  return hardware_interface::return_type::OK;
}

std::vector<hardware_interface::StateInterface> BearSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_states_velocities_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_states_efforts_[i]));
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> BearSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_positions_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_velocities_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_commands_efforts_[i]));
  }
  return command_interfaces;
}

} // namespace bear_hardware_interface

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(bear_hardware_interface::BearSystemHardware, hardware_interface::SystemInterface)