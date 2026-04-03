cmake_minimum_required(VERSION 3.8)
project(bear_hardware_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

# --- CONFIGURACIÓN CBEAR ---
# Incluimos los directorios que copiamos
include_directories(
  include
  external/CBEAR/include
)

# Compilamos la librería CBEAR primero (solo los .cpp necesarios)
add_library(cbear_sdk STATIC
  external/CBEAR/src/bear_sdk.cpp
  external/CBEAR/src/packet_manager.cpp
  external/CBEAR/src/port_manager.cpp
)
set_target_properties(cbear_sdk PROPERTIES POSITION_INDEPENDENT_CODE ON)
# ---------------------------

add_library(${PROJECT_NAME} SHARED
  src/bear_system.cpp
)

target_link_libraries(${PROJECT_NAME}
  PRIVATE
  cbear_sdk
  hardware_interface::hardware_interface
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
)

# Instalar el plugin
pluginlib_export_plugin_description_file(hardware_interface bear_hardware_plugins.xml)

install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(
  FILES bear_hardware_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY description launch config
  DESTINATION share/${PROJECT_NAME}
)
ament_package()