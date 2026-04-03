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
          // --- SEGURIDAD Y PID DE CORRIENTE ---
          // Limite máximo de corriente (iq_max = 3.0A)
          bear_instance_->SetLimitIMax(id, 3.0f);

          // PID de Corriente Iq (Eje de torque)
          bear_instance_->SetPGainIq(id, 0.02f);
          bear_instance_->SetIGainIq(id, 0.02f);
          bear_instance_->SetDGainIq(id, 0.0f);

          // PID de Corriente Id (Eje de flujo)
          bear_instance_->SetPGainId(id, 0.02f);
          bear_instance_->SetIGainId(id, 0.02f);
          bear_instance_->SetDGainId(id, 0.0f);
          
          RCLCPP_INFO(rclcpp::get_logger("BearSystemHardware"), "Motor %d: Configuración de corriente lista.", id);
      }
  } catch (...) {
    RCLCPP_FATAL(rclcpp::get_logger("BearSystemHardware"), "Error al inicializar comunicación o registros.");
    return hardware_interface::CallbackReturn::ERROR;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn BearSystemHardware::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("BearSystemHardware"), "Entrando en CONTROL DE POSICIÓN (Modo 2)...");
  
  for (size_t i = 0; i < motor_ids_.size(); i++) {
      int id = motor_ids_[i];

      // 1. Setear Modo Posición
      bear_instance_->SetMode(id, 2);

      // 2. Aplicar Ganancias de Posición (Spring stiffness y Damper)
      // p_gain = 5.0, d_gain = 0.2, i_gain = 0.0
      bear_instance_->SetPGainPosition(id, 5.0f);
      bear_instance_->SetIGainPosition(id, 0.0f);
      bear_instance_->SetDGainPosition(id, 0.2f);

      // 3. Sincronizar posición actual para que el robot no salte al activar
      float current_pos = bear_instance_->GetPresentPosition(id);
      hw_states_positions_[i] = current_pos;
      hw_commands_positions_[i] = current_pos;

      // 4. Habilitar Torque
      bear_instance_->SetTorqueEnable(id, 1);
      
      RCLCPP_INFO(rclcpp::get_logger("BearSystemHardware"), "Motor %d ACTIVADO en modo posición.", id);
  }
  
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn BearSystemHardware::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("BearSystemHardware"), "Deshabilitando Motores...");
  for (int id : motor_ids_) {
      bear_instance_->SetTorqueEnable(id, 0);
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn BearSystemHardware::on_cleanup(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("BearSystemHardware"), "Limpiando...");
  bear_instance_.reset();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn BearSystemHardware::on_shutdown(const rclcpp_lifecycle::State & /*previous_state*/)
{
  for (int id : motor_ids_) {
      bear_instance_->SetTorqueEnable(id, 0);
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type BearSystemHardware::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  const double Kt = 0.35; // Constante par-corriente

  for (size_t i = 0; i < motor_ids_.size(); i++) {
      hw_states_positions_[i] = bear_instance_->GetPresentPosition(motor_ids_[i]);
      hw_states_velocities_[i] = bear_instance_->GetPresentVelocity(motor_ids_[i]);
      
      // Leemos Iq (corriente) y convertimos a Torque (Nm)
      float iq = bear_instance_->GetPresentIq(motor_ids_[i]);
      hw_states_efforts_[i] = iq * Kt; 
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type BearSystemHardware::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  for (size_t i = 0; i < motor_ids_.size(); i++) {
      if (!std::isnan(hw_commands_positions_[i])) {
          bear_instance_->SetGoalPosition(motor_ids_[i], (float)hw_commands_positions_[i]);
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
    // AÑADIDO: Exportar el esfuerzo que se está leyendo en read()
    state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_states_efforts_[i]));
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> BearSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_positions_[i]));
  }
  return command_interfaces;
}

} // namespace bear_hardware_interface <-- CORREGIDO: Faltaba cerrar el namespace adecuadamente

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(bear_hardware_interface::BearSystemHardware, hardware_interface::SystemInterface)