// Copyright 2024 ROBOTIS CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Authors: Hye-Jong KIM, Sungho Woo, Woojin Wie

#include "dynamixel_hardware_interface/dynamixel_hardware_interface.hpp"

#include <atomic>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>
#include <string>
#include <map>
#include <unordered_map>
#include <iomanip>
#include <sstream>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace dynamixel_hardware_interface
{

namespace
{

std::string make_internal_node_name()
{
  static std::atomic<unsigned int> next_instance_id{0};
  return "instance_" + std::to_string(next_instance_id.fetch_add(1));
}

}  // namespace

DynamixelHardware::DynamixelHardware()
: rclcpp::Node(
    make_internal_node_name(),
    "/dynamixel_hardware_interface"),
  logger_(rclcpp::Node::get_logger())
{
  dxl_status_ = DXL_OK;
  dxl_torque_status_ = TORQUE_ENABLED;
  err_timeout_ms_ = 500;
  is_read_in_error_ = false;
  is_write_in_error_ = false;
  read_error_duration_ = rclcpp::Duration(0, 0);
  write_error_duration_ = rclcpp::Duration(0, 0);
}

DynamixelHardware::~DynamixelHardware()
{
  stop();

  if (rclcpp::ok()) {
    RCLCPP_INFO(logger_, "Shutting down ROS2 node...");
  }
}

hardware_interface::CallbackReturn DynamixelHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    RCLCPP_ERROR_STREAM(logger_, "Failed to initialize DynamixelHardware");
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (info_.hardware_parameters.find("number_of_joints") == info_.hardware_parameters.end()) {
    RCLCPP_ERROR_STREAM(
      logger_,
      "Required parameter 'number_of_joints' not found in hardware parameters");
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (info_.hardware_parameters.find("number_of_transmissions") ==
    info_.hardware_parameters.end())
  {
    RCLCPP_ERROR_STREAM(
      logger_,
      "Required parameter 'number_of_transmissions' not found in hardware parameters");
    return hardware_interface::CallbackReturn::ERROR;
  }

  num_of_joints_ = static_cast<size_t>(stoi(info_.hardware_parameters["number_of_joints"]));
  num_of_transmissions_ =
    static_cast<size_t>(stoi(info_.hardware_parameters["number_of_transmissions"]));

  if (!SetMatrix()) {
    RCLCPP_ERROR_STREAM(logger_, "Failed to set transmission matrices");
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (info_.hardware_parameters.find("port_name") == info_.hardware_parameters.end()) {
    RCLCPP_ERROR_STREAM(logger_, "Required parameter 'port_name' not found in hardware parameters");
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (info_.hardware_parameters.find("baud_rate") == info_.hardware_parameters.end()) {
    RCLCPP_ERROR_STREAM(logger_, "Required parameter 'baud_rate' not found in hardware parameters");
    return hardware_interface::CallbackReturn::ERROR;
  }

  port_name_ = info_.hardware_parameters["port_name"];
  baud_rate_ = info_.hardware_parameters["baud_rate"];
  if (info_.hardware_parameters.find("error_timeout_ms") != info_.hardware_parameters.end()) {
    try {
      err_timeout_ms_ = stod(info_.hardware_parameters["error_timeout_ms"]);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(
        logger_, "Failed to parse error_timeout_ms parameter: %s, using default value",
        e.what());
    }
  } else {
    RCLCPP_INFO(logger_, "error_timeout_ms parameter not found, using default value of 500ms");
  }

  // Asynchronous read/write: run blocking serial I/O in a background thread so
  // that multiple hardware components (e.g. one serial bus per limb) communicate
  // in parallel instead of sequentially inside the controller_manager thread.
  if (info_.hardware_parameters.find("use_async_read_write") != info_.hardware_parameters.end()) {
    use_async_ = info_.hardware_parameters.at("use_async_read_write") == "true";
  }
  if (info_.hardware_parameters.find("async_period_ms") != info_.hardware_parameters.end()) {
    try {
      async_period_ms_ = stod(info_.hardware_parameters.at("async_period_ms"));
    } catch (const std::exception & e) {
      RCLCPP_ERROR(
        logger_, "Failed to parse async_period_ms parameter: %s, using default value", e.what());
    }
  }
  if (!std::isfinite(async_period_ms_) || async_period_ms_ < 0.0) {
    async_period_ms_ = 1.0;
  }
  RCLCPP_INFO(
    logger_, "Asynchronous read/write is %s%s", use_async_ ? "ENABLED" : "DISABLED",
    use_async_ ? (" (thread period " + std::to_string(async_period_ms_) + " ms)").c_str() : "");

  // Add new parameter for torque initialization
  bool disable_torque_at_init = false;
  if (info_.hardware_parameters.find("disable_torque_at_init") != info_.hardware_parameters.end()) {
    disable_torque_at_init = info_.hardware_parameters.at("disable_torque_at_init") == "true";
    if (disable_torque_at_init) {
      RCLCPP_INFO(
        logger_,
        "Torque will be disabled during initialization if it is enabled at initialization.");
    }
  } else {
    RCLCPP_INFO(
      logger_,
      "If there is a torque enabled Dynamixel, the program will be terminated. Set "
      "'disable_torque_at_init' parameter to 'true' to disable torque at initialization.");
  }

  RCLCPP_INFO_STREAM(
    logger_,
    "port_name " << port_name_.c_str() << " / baudrate " << baud_rate_.c_str());

  if (info_.hardware_parameters.find("dynamixel_model_folder") == info_.hardware_parameters.end()) {
    RCLCPP_ERROR_STREAM(
      logger_,
      "Required parameter 'dynamixel_model_folder' not found in hardware parameters");
    return hardware_interface::CallbackReturn::ERROR;
  }
  std::string dxl_model_folder = info_.hardware_parameters["dynamixel_model_folder"];
  const bool enable_debug =
    info_.hardware_parameters.find("enable_debug") != info_.hardware_parameters.end() &&
    info_.hardware_parameters.at("enable_debug") == "true";
  dxl_comm_ = std::unique_ptr<Dynamixel>(
    new Dynamixel(
      (ament_index_cpp::get_package_share_directory("dynamixel_hardware_interface") +
      dxl_model_folder).c_str(),
      enable_debug));

  RCLCPP_INFO_STREAM(logger_, "$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$");
  RCLCPP_INFO_STREAM(logger_, "$$$$$ Init Dxl Comm Port");

  if (info_.hardware_parameters.find("use_revolute_to_prismatic_gripper") !=
    info_.hardware_parameters.end())
  {
    use_revolute_to_prismatic_ =
      std::stoi(info_.hardware_parameters.at("use_revolute_to_prismatic_gripper")) != 0;
  }

  if (use_revolute_to_prismatic_) {
    RCLCPP_INFO(logger_, "Revolute to Prismatic gripper conversion enabled.");
    initRevoluteToPrismaticParam();
  }
  for (const hardware_interface::ComponentInfo & gpio : info_.gpios) {
    if (gpio.parameters.find("ID") == gpio.parameters.end()) {
      RCLCPP_ERROR_STREAM(logger_, "ID is not found in gpio parameters");
      exit(-1);
    }
    if (gpio.parameters.find("type") == gpio.parameters.end()) {
      RCLCPP_ERROR_STREAM(logger_, "type is not found in gpio parameters");
      exit(-1);
    }

    uint8_t id = static_cast<uint8_t>(stoi(gpio.parameters.at("ID")));
    uint8_t comm_id = (gpio.parameters.find("comm_id") != gpio.parameters.end()) ?
      static_cast<uint8_t>(stoi(gpio.parameters.at("comm_id"))) : id;
    const std::string & type = gpio.parameters.at("type");

    if (type == "dxl") {
      dxl_comm_id_id_.emplace_back(comm_id, id);
    } else if (type == "sensor") {
      sensor_comm_id_id_.emplace_back(comm_id, id);
    } else if (type == "controller") {
      controller_comm_id_id_.emplace_back(comm_id, id);
    } else if (type == "virtual_dxl") {
      dxl_comm_->ReadDxlModelFile(
        comm_id, id,
        static_cast<uint16_t>(stoi(gpio.parameters.at("model_num"))));
      virtual_dxl_comm_id_id_.emplace_back(comm_id, id);
    } else if (type == "virtual_sensor") {
      dxl_comm_->ReadDxlModelFile(
        comm_id, id,
        static_cast<uint16_t>(stoi(gpio.parameters.at("model_num"))));
    } else {
      RCLCPP_ERROR_STREAM(logger_, "Invalid DXL / Sensor type");
      exit(-1);
    }
  }

  if (dxl_comm_->SetupPort(port_name_, baud_rate_) != DxlError::OK) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  std::vector<uint8_t> reboot_dxl;
  for (const hardware_interface::ComponentInfo & gpio : info_.gpios) {
    if (gpio.parameters.find("Reboot") != gpio.parameters.end() &&
      std::stoi(gpio.parameters.at("Reboot")) != 0)
    {
      reboot_dxl.push_back(static_cast<uint8_t>(stoi(gpio.parameters.at("ID"))));
    }
  }
  for (uint8_t id : reboot_dxl) {
    for (int attempt = 0; attempt < 5; ++attempt) {
      if (dxl_comm_->Reboot(id) == DxlError::OK) {
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(250));
    }
  }

  torque_enabled_comm_id_id_.clear();
  for (const hardware_interface::ComponentInfo & gpio : info_.gpios) {
    uint8_t id = static_cast<uint8_t>(stoi(gpio.parameters.at("ID")));
    uint8_t comm_id = (gpio.parameters.find("comm_id") != gpio.parameters.end()) ?
      static_cast<uint8_t>(stoi(gpio.parameters.at("comm_id"))) : id;
    const std::string & type = gpio.parameters.at("type");

    bool requires_ping = (type == "controller" || type == "dxl" || type == "sensor");
    if (requires_ping) {
      bool success = false;
      for (int attempt = 0; attempt < 10; ++attempt) {
        if (dxl_comm_->InitDxlComm(comm_id, id) == DxlError::OK) {
          success = true;
          break;
        }
        std::this_thread::sleep_for(std::chrono::seconds(1));
      }
      if (!success) {
        RCLCPP_ERROR_STREAM(
          logger_,
          "[comm_id:" << static_cast<int>(comm_id) <<
            "][ID:" << static_cast<int>(id) <<
            "] Cannot connect communication port! :(");
        return hardware_interface::CallbackReturn::ERROR;
      }
    }

    if (type == "dxl" || type == "virtual_dxl") {
      std::vector<std::pair<uint8_t, uint8_t>> single_pair = {{comm_id, id}};
      if (dxl_comm_->InitTorqueStates(single_pair, disable_torque_at_init) != DxlError::OK) {
        RCLCPP_ERROR_STREAM(
          logger_,
          "[comm_id:" << static_cast<int>(comm_id) <<
            "][ID:" << static_cast<int>(id) << "] Error: InitTorqueStates");
        return hardware_interface::CallbackReturn::ERROR;
      }
    }

    if (!InitItem(gpio)) {
      RCLCPP_ERROR_STREAM(
        logger_,
        "[comm_id:" << static_cast<int>(comm_id) <<
          "][ID:" << static_cast<int>(id) << "] Error: InitItem");
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  if (!InitDxlReadItems()) {
    RCLCPP_ERROR_STREAM(logger_, "Error: InitDxlReadItems");
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!InitDxlWriteItems()) {
    RCLCPP_ERROR_STREAM(logger_, "Error: InitDxlWriteItems");
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (num_of_transmissions_ != hdl_trans_commands_.size() &&
    num_of_transmissions_ != hdl_trans_states_.size())
  {
    RCLCPP_ERROR_STREAM(
      logger_, "Error: number of transmission " << num_of_transmissions_ << ", " <<
        hdl_trans_commands_.size() << ", " << hdl_trans_states_.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  dxl_status_ = DXL_OK;

  hdl_joint_states_.clear();
  for (const hardware_interface::ComponentInfo & joint : info_.joints) {
    HandlerVarType temp_state;
    temp_state.name = joint.name;

    for (auto it : joint.state_interfaces) {
      if (hardware_interface::HW_IF_POSITION != it.name &&
        hardware_interface::HW_IF_VELOCITY != it.name &&
        hardware_interface::HW_IF_ACCELERATION != it.name &&
        hardware_interface::HW_IF_EFFORT != it.name &&
        HW_IF_HARDWARE_STATE != it.name &&
        HW_IF_TORQUE_ENABLE != it.name)
      {
        RCLCPP_ERROR_STREAM(
          logger_, "Error: invalid joint state interface " << it.name);
        return hardware_interface::CallbackReturn::ERROR;
      }
      temp_state.interface_name_vec.push_back(it.name);
      temp_state.value_ptr_vec.push_back(std::make_shared<double>(0.0));
    }
    hdl_joint_states_.push_back(temp_state);
  }

  hdl_joint_commands_.clear();
  for (const hardware_interface::ComponentInfo & joint : info_.joints) {
    HandlerVarType temp_cmd;
    temp_cmd.name = joint.name;

    for (auto it : joint.command_interfaces) {
      if (hardware_interface::HW_IF_POSITION != it.name &&
        hardware_interface::HW_IF_VELOCITY != it.name &&
        hardware_interface::HW_IF_ACCELERATION != it.name &&
        hardware_interface::HW_IF_EFFORT != it.name)
      {
        RCLCPP_ERROR_STREAM(
          logger_, "Error: invalid joint command interface " << it.name);
        return hardware_interface::CallbackReturn::ERROR;
      }
      temp_cmd.interface_name_vec.push_back(it.name);
      temp_cmd.value_ptr_vec.push_back(std::make_shared<double>(0.0));
    }
    hdl_joint_commands_.push_back(temp_cmd);
  }

  if (num_of_joints_ != hdl_joint_commands_.size() &&
    num_of_joints_ != hdl_joint_states_.size())
  {
    RCLCPP_ERROR_STREAM(
      logger_, "Error: number of joints " << num_of_joints_ << ", " <<
        hdl_joint_commands_.size() << ", " << hdl_joint_commands_.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  hdl_sensor_states_.clear();
  for (const hardware_interface::ComponentInfo & sensor : info_.sensors) {
    HandlerVarType temp_state;
    temp_state.name = sensor.name;

    for (auto it : sensor.state_interfaces) {
      temp_state.interface_name_vec.push_back(it.name);
      temp_state.value_ptr_vec.push_back(std::make_shared<double>(0.0));
    }
    hdl_sensor_states_.push_back(temp_state);
  }

  std::string str_dxl_state_pub_name = "dxl_state";
  if (info_.hardware_parameters.find("dynamixel_state_pub_msg_name") !=
    info_.hardware_parameters.end())
  {
    str_dxl_state_pub_name = info_.hardware_parameters["dynamixel_state_pub_msg_name"];
  }
  dxl_state_pub_ = this->create_publisher<DynamixelStateMsg>(
    str_dxl_state_pub_name, rclcpp::SystemDefaultsQoS());
  dxl_state_pub_uni_ptr_ = std::make_unique<StatePublisher>(dxl_state_pub_);

  size_t num_of_pub_data = hdl_trans_states_.size();
  dxl_state_pub_uni_ptr_->lock();
  dxl_state_pub_uni_ptr_->msg_.id.resize(num_of_pub_data);
  dxl_state_pub_uni_ptr_->msg_.dxl_hw_state.resize(num_of_pub_data);
  dxl_state_pub_uni_ptr_->msg_.torque_state.resize(num_of_pub_data);
  dxl_state_pub_uni_ptr_->unlock();

  using namespace std::placeholders;

  // Get Dynamixel data service
  std::string str_get_dxl_data_srv_name = "get_dxl_data";
  if (info_.hardware_parameters.find("get_dynamixel_data_srv_name") !=
    info_.hardware_parameters.end())
  {
    str_get_dxl_data_srv_name = info_.hardware_parameters["get_dynamixel_data_srv_name"];
  }
  get_dxl_data_srv_ = create_service<dynamixel_interfaces::srv::GetDataFromDxl>(
    str_get_dxl_data_srv_name,
    std::bind(&DynamixelHardware::get_dxl_data_srv_callback, this, _1, _2));

  // Set Dynamixel data service
  std::string str_set_dxl_data_srv_name = "set_dxl_data";
  if (info_.hardware_parameters.find("set_dynamixel_data_srv_name") !=
    info_.hardware_parameters.end())
  {
    str_set_dxl_data_srv_name = info_.hardware_parameters["set_dynamixel_data_srv_name"];
  }
  set_dxl_data_srv_ = create_service<dynamixel_interfaces::srv::SetDataToDxl>(
    str_set_dxl_data_srv_name,
    std::bind(&DynamixelHardware::set_dxl_data_srv_callback, this, _1, _2));

  // Reboot Dynamixel service
  std::string str_reboot_dxl_srv_name = "reboot_dxl";
  if (info_.hardware_parameters.find("reboot_dxl_srv_name") != info_.hardware_parameters.end()) {
    str_reboot_dxl_srv_name = info_.hardware_parameters["reboot_dxl_srv_name"];
  }
  reboot_dxl_srv_ = create_service<dynamixel_interfaces::srv::RebootDxl>(
    str_reboot_dxl_srv_name,
    std::bind(&DynamixelHardware::reboot_dxl_srv_callback, this, _1, _2));

  // Set Dynamixel torque service
  std::string str_set_dxl_torque_srv_name = "set_dxl_torque";
  if (info_.hardware_parameters.find("set_dxl_torque_srv_name") !=
    info_.hardware_parameters.end())
  {
    str_set_dxl_torque_srv_name = info_.hardware_parameters["set_dxl_torque_srv_name"];
  }
  set_dxl_torque_srv_ = create_service<std_srvs::srv::SetBool>(
    str_set_dxl_torque_srv_name,
    std::bind(&DynamixelHardware::set_dxl_torque_srv_callback, this, _1, _2));

  return hardware_interface::CallbackReturn::SUCCESS;
}

double * DynamixelHardware::exportedValue(
  AsyncInterfaceBuffer & buffer, const HandlerVarType & handler, size_t index)
{
  const auto & value = handler.value_ptr_vec.at(index);
  if (!use_async_) {
    return value.get();
  }
  return buffer.bind(handler.name + "/" + handler.interface_name_vec.at(index), value);
}

void DynamixelHardware::bindAsyncInterfaces()
{
  // Rebind raw GPIO storage after communication handlers have been rebuilt.
  for (const auto * group : {&hdl_trans_states_, &hdl_joint_states_, &hdl_sensor_states_,
      &hdl_gpio_controller_states_})
  {
    for (const auto & handler : *group) {
      for (size_t i = 0; i < handler.value_ptr_vec.size(); ++i) {
        exportedValue(async_states_, handler, i);
      }
    }
  }
  for (const auto * group : {&hdl_trans_commands_, &hdl_joint_commands_,
      &hdl_gpio_controller_commands_})
  {
    for (const auto & handler : *group) {
      for (size_t i = 0; i < handler.value_ptr_vec.size(); ++i) {
        exportedValue(async_commands_, handler, i);
      }
    }
  }
}

std::vector<hardware_interface::StateInterface>
DynamixelHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  try {
    for (auto it : hdl_trans_states_) {
      for (size_t i = 0; i < it.value_ptr_vec.size(); i++) {
        if (i >= it.interface_name_vec.size()) {
          RCLCPP_ERROR_STREAM(
            logger_,
            "Interface name vector size mismatch for " << it.name <<
              ". Expected size: " << it.value_ptr_vec.size() <<
              ", Actual size: " << it.interface_name_vec.size());
          continue;
        }
        state_interfaces.emplace_back(
          hardware_interface::StateInterface(
            it.name, it.interface_name_vec.at(i), exportedValue(async_states_, it, i)));
      }
    }
    for (auto it : hdl_joint_states_) {
      for (size_t i = 0; i < it.value_ptr_vec.size(); i++) {
        if (i >= it.interface_name_vec.size()) {
          RCLCPP_ERROR_STREAM(
            logger_, "Interface name vector size mismatch for joint " << it.name <<
              ". Expected size: " << it.value_ptr_vec.size() <<
              ", Actual size: " << it.interface_name_vec.size());
          continue;
        }
        state_interfaces.emplace_back(
          hardware_interface::StateInterface(
            it.name, it.interface_name_vec.at(i), exportedValue(async_states_, it, i)));
      }
    }
    for (auto it : hdl_sensor_states_) {
      for (size_t i = 0; i < it.value_ptr_vec.size(); i++) {
        if (i >= it.interface_name_vec.size()) {
          RCLCPP_ERROR_STREAM(
            logger_, "Interface name vector size mismatch for sensor " << it.name <<
              ". Expected size: " << it.value_ptr_vec.size() <<
              ", Actual size: " << it.interface_name_vec.size());
          continue;
        }
        state_interfaces.emplace_back(
          hardware_interface::StateInterface(
            it.name, it.interface_name_vec.at(i), exportedValue(async_states_, it, i)));
      }
    }
    for (auto it : hdl_gpio_controller_states_) {
      for (size_t i = 0; i < it.value_ptr_vec.size(); i++) {
        if (i >= it.interface_name_vec.size()) {
          RCLCPP_ERROR_STREAM(
            logger_, "Interface name vector size mismatch for gpio controller " << it.name <<
              ". Expected size: " << it.value_ptr_vec.size() <<
              ", Actual size: " << it.interface_name_vec.size());
          continue;
        }
        state_interfaces.emplace_back(
          hardware_interface::StateInterface(
            it.name, it.interface_name_vec.at(i), exportedValue(async_states_, it, i)));
      }
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR_STREAM(logger_, "Error in export_state_interfaces: " << e.what());
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
DynamixelHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  try {
    for (auto it : hdl_trans_commands_) {
      for (size_t i = 0; i < it.value_ptr_vec.size(); i++) {
        if (i >= it.interface_name_vec.size()) {
          RCLCPP_ERROR_STREAM(
            logger_, "Interface name vector size mismatch for " << it.name <<
              ". Expected size: " << it.value_ptr_vec.size() <<
              ", Actual size: " << it.interface_name_vec.size());
          continue;
        }
        command_interfaces.emplace_back(
          hardware_interface::CommandInterface(
            it.name, it.interface_name_vec.at(i), exportedValue(async_commands_, it, i)));
      }
    }
    for (auto it : hdl_joint_commands_) {
      for (size_t i = 0; i < it.value_ptr_vec.size(); i++) {
        if (i >= it.interface_name_vec.size()) {
          RCLCPP_ERROR_STREAM(
            logger_, "Interface name vector size mismatch for joint " << it.name <<
              ". Expected size: " << it.value_ptr_vec.size() <<
              ", Actual size: " << it.interface_name_vec.size());
          continue;
        }
        command_interfaces.emplace_back(
          hardware_interface::CommandInterface(
            it.name, it.interface_name_vec.at(i), exportedValue(async_commands_, it, i)));
      }
    }
    for (auto it : hdl_gpio_controller_commands_) {
      for (size_t i = 0; i < it.value_ptr_vec.size(); i++) {
        if (i >= it.interface_name_vec.size()) {
          RCLCPP_ERROR_STREAM(
            logger_, "Interface name vector size mismatch for gpio controller " <<
              it.name << ". Expected size: " << it.value_ptr_vec.size() <<
              ", Actual size: " << it.interface_name_vec.size());
          continue;
        }
        command_interfaces.emplace_back(
          hardware_interface::CommandInterface(
            it.name, it.interface_name_vec.at(i), exportedValue(async_commands_, it, i)));
      }
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR_STREAM(logger_, "Error in export_command_interfaces: " << e.what());
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn DynamixelHardware::on_activate(
  [[maybe_unused]] const rclcpp_lifecycle::State & previous_state)
{
  return start();
}

hardware_interface::CallbackReturn DynamixelHardware::on_deactivate(
  [[maybe_unused]] const rclcpp_lifecycle::State & previous_state)
{
  return stop();
}

hardware_interface::CallbackReturn DynamixelHardware::start()
{
  // Lifecycle setup must not modify the handlers while an old worker still owns them.
  comm_thread_running_.store(false, std::memory_order_release);
  if (comm_thread_.joinable()) {
    comm_thread_.join();
  }
  if (use_async_) {
    bindAsyncInterfaces();
  }

  async_read_status_.store(hardware_interface::return_type::ERROR, std::memory_order_release);
  async_write_status_.store(hardware_interface::return_type::ERROR, std::memory_order_release);
  const auto fail_start = [this](const std::string & reason) {
      RCLCPP_ERROR_STREAM(logger_, "Dynamixel Hardware Start aborted: " << reason);
      // Do not trust the torque cache, including after a partially acknowledged enable.
      const auto real = dxl_comm_->DynamixelDisable(dxl_comm_id_id_);
      const auto virtual_result = dxl_comm_->DynamixelDisable(virtual_dxl_comm_id_id_);
      if (real != DxlError::OK || virtual_result != DxlError::OK) {
        RCLCPP_ERROR_STREAM(logger_, "Torque OFF could not be confirmed on all devices.");
      } else {
        dxl_torque_status_ = TORQUE_DISABLED;
      }
      dxl_status_ = COMM_ERROR;
      return hardware_interface::CallbackReturn::ERROR;
    };

  // Startup owns the bus until all checks pass; controllers and the async worker remain inactive.
  const auto real_off = dxl_comm_->DynamixelDisable(dxl_comm_id_id_);
  const auto virtual_off = dxl_comm_->DynamixelDisable(virtual_dxl_comm_id_id_);
  if (real_off != DxlError::OK || virtual_off != DxlError::OK) {
    return fail_start("initial torque OFF was not confirmed");
  }

  const auto start_time = std::chrono::steady_clock::now();
  do {
    dxl_comm_err_ = CheckError(dxl_comm_->ReadStartupState());
    if (dxl_comm_err_ == DxlError::OK) {
      break;
    }
  } while (std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - start_time).count() < err_timeout_ms_);
  if (dxl_comm_err_ != DxlError::OK) {
    return fail_start("initial state read failed: " + Dynamixel::DxlErrorToString(dxl_comm_err_));
  }

  for (const auto & gpio : info_.gpios) {
    const auto mode = gpio.parameters.find("Operating Mode");
    if (mode == gpio.parameters.end()) {
      continue;
    }
    const uint8_t id = static_cast<uint8_t>(std::stoi(gpio.parameters.at("ID")));
    const auto comm = gpio.parameters.find("comm_id");
    const uint8_t comm_id = comm == gpio.parameters.end() ? id : std::stoi(comm->second);
    uint32_t actual_mode = 0;
    if (dxl_comm_->ReadItem(comm_id, id, "Operating Mode", actual_mode, false) != DxlError::OK ||
      actual_mode != static_cast<uint32_t>(std::stoi(mode->second)))
    {
      return fail_start(gpio.name + ": operating mode was not confirmed");
    }
    const auto required = actual_mode == 1 ? "Goal Velocity" : "Goal Position";
    if (std::find(
        torque_enabled_comm_id_id_.begin(), torque_enabled_comm_id_id_.end(),
        std::make_pair(comm_id, id)) != torque_enabled_comm_id_id_.end() &&
      (actual_mode == 1 || actual_mode == 3 || actual_mode == 4 || actual_mode == 5))
    {
      const auto command = std::find_if(
        gpio.command_interfaces.begin(), gpio.command_interfaces.end(),
        [required](const hardware_interface::InterfaceInfo & interface) {
          return interface.name == required;
        });
      if (command == gpio.command_interfaces.end()) {
        return fail_start(gpio.name + ": no stop/hold command interface for operating mode");
      }
    }
  }

  applyMultiTurnOffsets();
  CalcTransmissionToJoint();
  // Zero retained velocity/effort commands on every activation, including a restart after failure.
  for (auto * handlers : {&hdl_joint_commands_, &hdl_trans_commands_,
      &hdl_gpio_controller_commands_})
  {
    for (auto & handler : *handlers) {
      for (auto & value : handler.value_ptr_vec) {
        *value = 0.0;
      }
    }
  }
  for (const auto & command : hdl_joint_commands_) {
    for (const auto & interface : command.interface_name_vec) {
      if (interface != hardware_interface::HW_IF_POSITION) {
        continue;
      }
      bool valid = false;
      for (const auto & state : hdl_joint_states_) {
        if (state.name != command.name) {continue;}
        for (size_t i = 0; i < state.interface_name_vec.size(); ++i) {
          if (state.interface_name_vec[i] == interface) {
            valid = std::isfinite(*state.value_ptr_vec[i]);
          }
        }
      }
      if (!valid) {
        return fail_start(command.name + ": no valid initial position");
      }
    }
  }
  SyncJointCommandWithStates();
  CalcJointToTransmission();
  removeMultiTurnOffsets();
  // Reject invalid mapped commands before unit conversion or transmission.
  for (const auto & command : hdl_trans_commands_) {
    for (size_t i = 0; i < command.interface_name_vec.size(); ++i) {
      const auto & interface = command.interface_name_vec[i];
      const double value = *command.value_ptr_vec[i];
      if (!std::isfinite(value)) {
        restoreMultiTurnOffsets();
        return fail_start(command.name + ": non-finite initial command");
      }
      if (interface == "Goal Velocity" || interface == "Goal Current" || interface == "Goal PWM") {
        if (value != 0.0) {
          restoreMultiTurnOffsets();
          return fail_start(command.name + ": initial motion command is not zero");
        }
      }
    }
  }
  dxl_comm_err_ = dxl_comm_->WriteMultiDxlData();
  if (dxl_comm_err_ == DxlError::OK) {
    dxl_comm_err_ = dxl_comm_->VerifyMultiDxlWrite();
  }
  restoreMultiTurnOffsets();
  if (dxl_comm_err_ != DxlError::OK) {
    return fail_start(
      "initial stop/hold command was not confirmed: " +
      Dynamixel::DxlErrorToString(dxl_comm_err_));
  }

  if (dxl_comm_->DynamixelEnable(torque_enabled_comm_id_id_) != DxlError::OK) {
    return fail_start("torque ON was not confirmed");
  }
  // Enabling torque may reset a device's position counter. Export a fresh state, not the old one.
  dxl_comm_err_ = CheckError(dxl_comm_->ReadStartupState());
  if (dxl_comm_err_ != DxlError::OK) {
    return fail_start("state read after torque ON failed");
  }
  applyMultiTurnOffsets();
  CalcTransmissionToJoint();
  SyncJointCommandWithStates();
  dxl_status_ = DXL_OK;
  dxl_torque_status_ = torque_enabled_comm_id_id_.empty() ? TORQUE_DISABLED : TORQUE_ENABLED;
  is_read_in_error_ = false;
  is_write_in_error_ = false;
  read_error_duration_ = rclcpp::Duration(0, 0);
  write_error_duration_ = rclcpp::Duration(0, 0);

  RCLCPP_INFO_STREAM(logger_, "Dynamixel Hardware Start!");

  if (use_async_) {
    // Seed the exported values from the completed initial read before controllers activate.
    async_states_.initialize();
    async_commands_.initialize();
    async_read_status_.store(hardware_interface::return_type::OK, std::memory_order_release);
    async_write_status_.store(hardware_interface::return_type::OK, std::memory_order_release);
    comm_thread_running_.store(true, std::memory_order_release);
    comm_thread_ = std::thread(&DynamixelHardware::asyncCommLoop, this);
    RCLCPP_INFO_STREAM(
      logger_, "Async read/write thread started (period " << async_period_ms_ << " ms).");
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DynamixelHardware::stop()
{
  // Stop the async worker first so no serial transfer runs concurrently with the
  // torque-disable sequence below or with dxl_comm_ destruction.
  if (comm_thread_running_.load(std::memory_order_acquire) || comm_thread_.joinable()) {
    comm_thread_running_.store(false, std::memory_order_release);
    if (comm_thread_.joinable()) {
      comm_thread_.join();
    }
    RCLCPP_INFO_STREAM(logger_, "Async read/write thread stopped.");
  }

  if (dxl_comm_) {
    const auto real = dxl_comm_->DynamixelDisable(dxl_comm_id_id_);
    const auto virtual_result = dxl_comm_->DynamixelDisable(virtual_dxl_comm_id_id_);
    if (real != DxlError::OK || virtual_result != DxlError::OK) {
      RCLCPP_ERROR_STREAM(logger_, "Dynamixel Hardware Stop: torque OFF was not confirmed.");
      return hardware_interface::CallbackReturn::ERROR;
    }
  } else {
    RCLCPP_ERROR_STREAM(logger_, "Dynamixel Hardware Stop Fail : dxl_comm_ is nullptr");
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO_STREAM(logger_, "Dynamixel Hardware Stop!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type DynamixelHardware::read(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  if (use_async_) {
    async_states_.readState();
    async_commands_.readCommandUpdates();
    return async_read_status_.load(std::memory_order_acquire);
  }
  return readCommunication(time, period);
}

hardware_interface::return_type DynamixelHardware::readCommunication(
  [[maybe_unused]] const rclcpp::Time & time, const rclcpp::Duration & period)
{
  if (dxl_status_ == REBOOTING) {
    // Hold the last state and report OK so the controller_manager keeps this and every other
    // limb's controller active while this bus reboots. Returning ERROR here would deactivate the
    // shared LimberoController and drop control of all limbs.
    RCLCPP_WARN_STREAM_THROTTLE(
      logger_, clock_, 1000, "Dynamixel rebooting: holding last state (control stays active).");
    return hardware_interface::return_type::OK;
  } else if (dxl_status_ == DXL_OK || dxl_status_ == COMM_ERROR || dxl_status_ == HW_ERROR) {
    dxl_comm_err_ = CheckError(dxl_comm_->ReadMultiDxlData());
    if (dxl_comm_err_ != DxlError::OK && dxl_comm_err_ != DxlError::DXL_HARDWARE_ERROR) {
      if (!is_read_in_error_) {
        is_read_in_error_ = true;
        read_error_duration_ = rclcpp::Duration(0, 0);
      }
      read_error_duration_ = read_error_duration_ + period;

      RCLCPP_ERROR_STREAM(
        logger_,
        "Dynamixel Read Fail (Duration: " << read_error_duration_.seconds() * 1000 << "ms/" <<
          err_timeout_ms_ << "ms)");

      if (read_error_duration_.seconds() * 1000 >= err_timeout_ms_) {
        return hardware_interface::return_type::ERROR;
      }
      return hardware_interface::return_type::OK;
    }
    is_read_in_error_ = false;
    read_error_duration_ = rclcpp::Duration(0, 0);
  }

  applyMultiTurnOffsets();

  CalcTransmissionToJoint();

  for (auto sensor : hdl_gpio_sensor_states_) {
    ReadSensorData(sensor);
  }

  dxl_comm_->ReadItemBuf();

  size_t index = 0;
  if (dxl_state_pub_uni_ptr_ && dxl_state_pub_uni_ptr_->trylock()) {
    dxl_state_pub_uni_ptr_->msg_.header.stamp = this->now();
    dxl_state_pub_uni_ptr_->msg_.comm_state = dxl_comm_err_;
    for (auto it : hdl_trans_states_) {
      dxl_state_pub_uni_ptr_->msg_.id.at(index) = it.id;
      dxl_state_pub_uni_ptr_->msg_.dxl_hw_state.at(index) = dxl_hw_err_[it.id];
      auto ts_it = dxl_torque_state_.find({it.comm_id, it.id});
      bool ts = (ts_it != dxl_torque_state_.end()) ? ts_it->second : false;
      dxl_state_pub_uni_ptr_->msg_.torque_state.at(index) = ts;
      index++;
    }
    dxl_state_pub_uni_ptr_->unlockAndPublish();
  }

  if (rclcpp::ok()) {
    rclcpp::spin_some(this->get_node_base_interface());
  }
  return hardware_interface::return_type::OK;
}
hardware_interface::return_type DynamixelHardware::write(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  if (use_async_) {
    async_commands_.writeCommands();
    return async_write_status_.load(std::memory_order_acquire);
  }
  return writeCommunication(time, period);
}

hardware_interface::return_type DynamixelHardware::writeCommunication(
  [[maybe_unused]] const rclcpp::Time & time, const rclcpp::Duration & period)
{
  if (dxl_status_ == REBOOTING) {
    // Skip writes and report OK while rebooting so the shared controller stays active.
    return hardware_interface::return_type::OK;
  }
  if (dxl_status_ == DXL_OK || dxl_status_ == HW_ERROR) {
    dxl_comm_->WriteItemBuf();

    ChangeDxlTorqueState();

    CalcJointToTransmission();

    removeMultiTurnOffsets();

    dxl_comm_->WriteMultiDxlData();

    restoreMultiTurnOffsets();

    is_write_in_error_ = false;
    write_error_duration_ = rclcpp::Duration(0, 0);

    return hardware_interface::return_type::OK;
  } else {
    write_error_duration_ = write_error_duration_ + period;

    RCLCPP_ERROR_STREAM(
      logger_,
      "Dynamixel Write Fail (Duration: " << write_error_duration_.seconds() * 1000 << "ms/" <<
        err_timeout_ms_ << "ms)");

    if (write_error_duration_.seconds() * 1000 >= err_timeout_ms_) {
      return hardware_interface::return_type::ERROR;
    }
    return hardware_interface::return_type::OK;
  }
}

void DynamixelHardware::asyncCommLoop()
{
  rclcpp::Time prev_time = this->now();

  while (comm_thread_running_.load(std::memory_order_relaxed)) {
    const rclcpp::Time now = this->now();
    rclcpp::Duration period = now - prev_time;
    prev_time = now;
    if (period.seconds() <= 0.0) {
      period = rclcpp::Duration::from_seconds(1e-6);
    }

    // Latch one complete command before I/O. A service handled during readCommunication()
    // may then synchronize a position command without having it overwritten before the write.
    async_commands_.applyCommands();
    const auto read_status = readCommunication(now, period);
    async_states_.captureState();
    async_read_status_.store(read_status, std::memory_order_release);
    const auto write_status = writeCommunication(now, period);
    async_write_status_.store(write_status, std::memory_order_release);

    // On a terminal error (comm failure exceeded err_timeout_ms) stop the worker
    // so it does not keep driving a bus the framework will deactivate. The cached
    // ERROR status is surfaced to the controller_manager on the next read()/write(),
    // which triggers the normal deactivation path (stop() joins this thread).
    if (read_status == hardware_interface::return_type::ERROR ||
      write_status == hardware_interface::return_type::ERROR)
    {
      RCLCPP_ERROR_STREAM(
        logger_, "Async read/write thread stopping due to terminal communication error.");
      break;
    }

    // Cap the loop rate to avoid a busy spin when a transfer returns quickly
    // (e.g. during REBOOTING). Serial I/O itself already paces the loop.
    if (async_period_ms_ > 0.0) {
      std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(async_period_ms_));
    }
  }
}

DxlError DynamixelHardware::CheckError(DxlError dxl_comm_err)
{
  DxlError error_state = DxlError::OK;
  dxl_status_ = DXL_OK;

  // check comm error
  if (dxl_comm_err != DxlError::OK) {
    RCLCPP_ERROR_STREAM_THROTTLE(
      logger_, clock_, 1000,
      "Communication Fail --> " << Dynamixel::DxlErrorToString(dxl_comm_err));
    dxl_status_ = COMM_ERROR;
    return dxl_comm_err;
  }

  // check hardware error
  for (size_t i = 0; i < num_of_transmissions_; i++) {
    for (size_t j = 0; j < hdl_trans_states_.at(i).interface_name_vec.size(); j++) {
      if (hdl_trans_states_.at(i).interface_name_vec.at(j) == "Hardware Error Status") {
        dxl_hw_err_[hdl_trans_states_.at(i).id] =
          static_cast<uint8_t>(*hdl_trans_states_.at(i).value_ptr_vec.at(j));
        uint8_t hw_error_status = static_cast<uint8_t>(dxl_hw_err_[hdl_trans_states_.at(i).id]);
        std::string error_string = "";

        // Check each bit in the hardware error status
        for (int bit = 0; bit < 8; ++bit) {
          if (hw_error_status & (1 << bit)) {
            const HardwareErrorStatusBitInfo * bit_info = get_hardware_error_status_bit_info(bit);
            if (bit_info) {
              error_string += bit_info->label;
              error_string += " (" + std::string(bit_info->description) + ")/ ";
            } else {
              error_string += "Unknown Error Bit " + std::to_string(bit) + "/ ";
            }
          }
        }

        if (!error_string.empty()) {
          RCLCPP_ERROR_STREAM_THROTTLE(
            logger_, clock_, 1000,
            "\033[1;31m[Dynamixel Hardware Error] ID:" <<
              static_cast<int>(hdl_trans_states_.at(i).id) << " --> " <<
              "0x" << std::hex << static_cast<int>(hw_error_status) << std::dec <<
              " (" << static_cast<int>(hw_error_status) << "): " << error_string << "\033[0m");
          dxl_status_ = HW_ERROR;
          error_state = DxlError::DXL_HARDWARE_ERROR;
        }
      }
      if (hdl_trans_states_.at(i).interface_name_vec.at(j) == "Error Code") {
        dxl_error_code_[hdl_trans_states_.at(i).id] =
          static_cast<uint8_t>(*hdl_trans_states_.at(i).value_ptr_vec.at(j));
        uint8_t error_code = static_cast<uint8_t>(dxl_error_code_[hdl_trans_states_.at(i).id]);

        if (error_code != 0x00) {
          const ErrorCodeInfo * error_info = get_error_code_info(error_code);
          if (error_info) {
            RCLCPP_ERROR_STREAM_THROTTLE(
              logger_, clock_, 1000,
              "\033[1;31m[Dynamixel Error Code] ID:" <<
                static_cast<int>(hdl_trans_states_.at(i).id) << " --> " <<
                "0x" << std::hex << static_cast<int>(error_code) << std::dec <<
                " (" << error_info->label << "): " << error_info->description << "\033[0m");
          } else {
            RCLCPP_ERROR_STREAM_THROTTLE(
              logger_, clock_, 1000,
              "\033[1;31m[Dynamixel Error Code] ID:" <<
                static_cast<int>(hdl_trans_states_.at(i).id) << " --> " <<
                "0x" << std::hex << static_cast<int>(error_code) << std::dec <<
                " (Unknown Error Code)\033[0m");
          }
          dxl_status_ = HW_ERROR;
          error_state = DxlError::DXL_HARDWARE_ERROR;
        }
      }
    }
  }

  for (size_t i = 0; i < num_of_joints_; i++) {
    for (size_t j = 0; j < hdl_joint_states_.at(i).interface_name_vec.size(); j++) {
      if (hdl_joint_states_.at(i).interface_name_vec.at(j) == HW_IF_HARDWARE_STATE) {
        *hdl_joint_states_.at(i).value_ptr_vec.at(j) = error_state;
      }
    }
  }

  return error_state;
}

bool DynamixelHardware::CommReset()
{
  dxl_status_ = REBOOTING;

  // Reboot only the dynamixel(s) currently flagged with an error; if none are flagged
  // (e.g. a manual reboot) fall back to rebooting every dynamixel on this bus.
  std::vector<std::pair<uint8_t, uint8_t>> reboot_targets;
  for (const auto & pr : dxl_comm_id_id_) {
    const uint8_t id = pr.second;
    const bool hw_err = dxl_hw_err_.count(id) && dxl_hw_err_.at(id) != 0;
    const bool code_err = dxl_error_code_.count(id) && dxl_error_code_.at(id) != 0x00;
    if (hw_err || code_err) {
      reboot_targets.push_back(pr);
      RCLCPP_ERROR_STREAM(
        logger_, "\033[1;31m[Reboot] Target ID " << static_cast<int>(id) << "\n"
                                                 << getErrorSummary(id) << "\033[0m");
    }
  }
  if (reboot_targets.empty()) {
    reboot_targets = dxl_comm_id_id_;
    RCLCPP_WARN_STREAM(
      logger_, "[Reboot] No error-flagged dynamixel found; rebooting all on this bus.");
  }

  last_known_positions_.clear();
  for (auto & state : hdl_trans_states_) {
    for (size_t i = 0; i < state.interface_name_vec.size(); ++i) {
      if (state.interface_name_vec[i] == "Present Position") {
        last_known_positions_[state.id] = *(state.value_ptr_vec[i]);
      }
    }
  }

  stop();
  RCLCPP_INFO_STREAM(logger_, "Communication Reset Start");
  dxl_comm_->RWDataReset();

  auto start_time = this->now();
  while ((this->now() - start_time) < rclcpp::Duration(3, 0)) {
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    RCLCPP_INFO_STREAM(logger_, "Reset Start");
    bool result = true;
    for (auto pr : reboot_targets) {
      if (dxl_comm_->Reboot(pr.first) != DxlError::OK) {
        RCLCPP_ERROR_STREAM(logger_, "\033[1;31mCannot reboot dynamixel! :(\033[0m");
        result = false;
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    if (!result) {continue;}
    // if (!InitControllerItems()) {continue;}
    // if (!InitDxlItems()) {continue;}
    if (!InitDxlReadItems()) {continue;}
    if (!InitDxlWriteItems()) {continue;}

    RCLCPP_INFO_STREAM(logger_, "RESET Success");
    std::this_thread::sleep_for(std::chrono::seconds(1));

    // Read the "raw zero-lap position" immediately after reboot and
    // calculate the number of laps out of sync
    dxl_comm_->ReadMultiDxlData();
    for (auto & state : hdl_trans_states_) {
      for (size_t i = 0; i < state.interface_name_vec.size(); ++i) {
        if (state.interface_name_vec[i] == "Present Position") {
          double raw_new_pos = *(state.value_ptr_vec[i]);
          if (last_known_positions_.find(state.id) != last_known_positions_.end()) {
            double last_pos = last_known_positions_[state.id];
            double diff = last_pos - raw_new_pos;
            double turns = std::round(diff / (2.0 * M_PI));
            multi_turn_offsets_[state.id] = turns * 2.0 * M_PI;
            RCLCPP_INFO_STREAM(
              logger_,
              "\033[32m [Multi-turn Offset] ID: " << (int)state.id
                                                  << " | Last: " << last_pos
                                                  << " | Raw: " << raw_new_pos
                                                  << " | Turns: " << turns << " | Offset: "
                                                  << multi_turn_offsets_[state.id] << " \033[0m");
          }
        }
      }
    }

    return start() == hardware_interface::CallbackReturn::SUCCESS;
  }
  RCLCPP_ERROR_STREAM(logger_, "\033[1;31m[Reboot] RESET Failure\033[0m");
  std::this_thread::sleep_for(std::chrono::seconds(1));
  start();
  return false;
}

bool DynamixelHardware::InitItem(const hardware_interface::ComponentInfo & gpio)
{
  uint8_t id = static_cast<uint8_t>(stoi(gpio.parameters.at("ID")));
  uint8_t comm_id = (gpio.parameters.find("comm_id") != gpio.parameters.end()) ?
    static_cast<uint8_t>(stoi(gpio.parameters.at("comm_id"))) : id;

  auto unit_it = gpio.parameters.find("[unit info]");
  if (unit_it != gpio.parameters.end()) {
    std::string value = unit_it->second;
    std::vector<std::string> entries;
    {
      std::stringstream ss(value);
      std::string token;
      while (std::getline(ss, token, ';')) {
        size_t first = token.find_first_not_of(" \t\r\n");
        if (first != std::string::npos) {
          size_t last = token.find_last_not_of(" \t\r\n");
          token = token.substr(first, last - first + 1);
        } else {
          token.clear();
        }
        if (!token.empty()) {entries.push_back(token);}
      }
      if (entries.empty()) {entries.push_back(value);}
    }
    for (auto & entry : entries) {
      std::stringstream ls(entry);
      std::string line;
      while (std::getline(ls, line)) {
        size_t first = line.find_first_not_of(" \t\r\n");
        if (first != std::string::npos) {
          size_t last = line.find_last_not_of(" \t\r\n");
          line = line.substr(first, last - first + 1);
        } else {
          line.clear();
        }
        if (line.empty()) {continue;}
        std::vector<std::string> parts;
        std::stringstream ps(line);
        std::string p;
        while (std::getline(ps, p, ',')) {
          size_t p_first = p.find_first_not_of(" \t\r\n");
          if (p_first != std::string::npos) {
            size_t p_last = p.find_last_not_of(" \t\r\n");
            p = p.substr(p_first, p_last - p_first + 1);
          } else {
            p.clear();
          }
          parts.push_back(p);
        }
        if (parts.size() < 4) {continue;}
        std::string data_name = parts[0];
        double unit_multiplier = 1.0;
        try {
          unit_multiplier = std::stod(parts[1]);
        } catch (...) {
          unit_multiplier = 1.0;
        }
        bool is_signed = (parts[3] == std::string("signed"));
        double offset = 0.0;
        if (parts.size() >= 5) {
          try {
            offset = std::stod(parts[4]);
          } catch (...) {
            offset = 0.0;
          }
        }
        dxl_comm_->OverrideUnitInfo(comm_id, id, data_name, unit_multiplier, is_signed, offset);
        RCLCPP_INFO_STREAM(
          logger_,
          "[ID:" << std::to_string(id) << "] override [unit info] for '" << data_name
                 << "' = multiplier:" << unit_multiplier << ", signed:" <<
            (is_signed ? "true" : "false")
                 << ", offset:" << offset);
      }
    }
  }

  bool torque_enabled =
    (gpio.parameters.find("type") != gpio.parameters.end() &&
    (gpio.parameters.at("type") == "dxl" || gpio.parameters.at("type") == "virtual_dxl"));
  if (gpio.parameters.find("Torque Enable") != gpio.parameters.end()) {
    torque_enabled = std::stoi(gpio.parameters.at("Torque Enable")) != 0;
  }
  if (torque_enabled) {
    torque_enabled_comm_id_id_.emplace_back(comm_id, id);
  }

  for (const auto & param : gpio.parameters) {
    const std::string & param_name = param.first;
    if (param_name == "Operating Mode") {
      RCLCPP_INFO_STREAM(
        logger_,
        "[InitItem][comm_id:" << std::to_string(comm_id) << "][ID:" << std::to_string(id) <<
          "] item_name: " << param_name.c_str() << "\tdata: " <<
          param.second);
      if (dxl_comm_->WriteItem(
          comm_id, id, param_name,
          static_cast<uint32_t>(stoi(param.second))) != DxlError::OK)
      {
        return false;
      }
    }
  }

  for (const auto & param : gpio.parameters) {
    const std::string & param_name = param.first;
    if (param_name == "ID" || param_name == "type" ||
      param_name == "Torque Enable" || param_name == "Operating Mode" ||
      param_name == "model_num" || param_name == "comm_id" || param_name == "Reboot")
    {
      continue;
    }
    if (param_name.find("Limit") != std::string::npos) {
      RCLCPP_INFO_STREAM(
        logger_,
        "[InitItem][comm_id:" << std::to_string(comm_id) << "][ID:" << std::to_string(id) <<
          "] item_name: " << param_name.c_str() << "\tdata: " <<
          param.second);
      if (dxl_comm_->WriteItem(
          comm_id, id, param_name,
          static_cast<uint32_t>(stoi(param.second))) != DxlError::OK)
      {
        return false;
      }
    }
  }

  for (const auto & param : gpio.parameters) {
    const std::string & param_name = param.first;
    if (param_name == "ID" || param_name == "type" ||
      param_name == "Torque Enable" || param_name == "Operating Mode" ||
      param_name == "model_num" || param_name == "comm_id" || param_name == "Reboot" ||
      param_name == "[unit info]" ||
      param_name.find("Limit") != std::string::npos)
    {
      continue;
    }
    RCLCPP_INFO_STREAM(
      logger_,
      "[InitItem][comm_id:" << std::to_string(comm_id) << "][ID:" << std::to_string(id) <<
        "] item_name: " << param_name.c_str() << "\tdata: " <<
        param.second);
    if (dxl_comm_->WriteItem(
        comm_id, id, param_name,
        static_cast<uint32_t>(stoi(param.second))) != DxlError::OK)
    {
      return false;
    }
  }
  return true;
}

bool DynamixelHardware::InitDxlReadItems()
{
  RCLCPP_INFO_STREAM(logger_, "$$$$$ Init Dxl Read Items");
  is_set_hdl_ = false;

  if (!is_set_hdl_) {
    hdl_trans_states_.clear();
    hdl_gpio_sensor_states_.clear();
    hdl_gpio_controller_states_.clear();
    for (const hardware_interface::ComponentInfo & gpio : info_.gpios) {
      if (gpio.parameters.at("type") == "dxl") {
        uint8_t id = static_cast<uint8_t>(stoi(gpio.parameters.at("ID")));
        HandlerVarType temp_read;
        temp_read.id = id;
        temp_read.comm_id = id;
        temp_read.name = gpio.name;

        for (auto it : gpio.state_interfaces) {
          temp_read.interface_name_vec.push_back(it.name);
          temp_read.value_ptr_vec.push_back(std::make_shared<double>(0.0));

          if (it.name == "Hardware Error Status") {
            dxl_hw_err_[id] = 0x00;
          }
        }
        hdl_trans_states_.push_back(temp_read);
      } else if (gpio.parameters.at("type") == "sensor") {
        uint8_t id = static_cast<uint8_t>(stoi(gpio.parameters.at("ID")));
        HandlerVarType temp_sensor;
        temp_sensor.id = id;
        temp_sensor.comm_id = id;
        temp_sensor.name = gpio.name;

        for (auto it : gpio.state_interfaces) {
          temp_sensor.interface_name_vec.push_back(it.name);
          temp_sensor.value_ptr_vec.push_back(std::make_shared<double>(0.0));
        }
        hdl_gpio_sensor_states_.push_back(temp_sensor);
      } else if (gpio.parameters.at("type") == "controller") {
        uint8_t id = static_cast<uint8_t>(stoi(gpio.parameters.at("ID")));
        HandlerVarType temp_controller;
        temp_controller.id = id;
        temp_controller.comm_id = id;
        temp_controller.name = gpio.name;

        for (auto it : gpio.state_interfaces) {
          temp_controller.interface_name_vec.push_back(it.name);
          temp_controller.value_ptr_vec.push_back(std::make_shared<double>(0.0));
        }
        hdl_gpio_controller_states_.push_back(temp_controller);
      } else if (gpio.parameters.at("type") == "virtual_dxl") {
        uint8_t id = static_cast<uint8_t>(stoi(gpio.parameters.at("ID")));
        uint8_t comm_id = static_cast<uint8_t>(stoi(gpio.parameters.at("comm_id")));
        HandlerVarType temp_read;
        temp_read.id = id;
        temp_read.comm_id = comm_id;
        temp_read.name = gpio.name;

        for (auto it : gpio.state_interfaces) {
          temp_read.interface_name_vec.push_back(it.name);
          temp_read.value_ptr_vec.push_back(std::make_shared<double>(0.0));

          if (it.name == "Hardware Error Status") {
            dxl_hw_err_[id] = 0x00;
          }
        }
        hdl_trans_states_.push_back(temp_read);
      } else if (gpio.parameters.at("type") == "virtual_sensor") {
        uint8_t id = static_cast<uint8_t>(stoi(gpio.parameters.at("ID")));
        uint8_t comm_id = static_cast<uint8_t>(stoi(gpio.parameters.at("comm_id")));
        HandlerVarType temp_sensor;
        temp_sensor.id = id;
        temp_sensor.comm_id = comm_id;
        temp_sensor.name = gpio.name;

        for (auto it : gpio.state_interfaces) {
          temp_sensor.interface_name_vec.push_back(it.name);
          temp_sensor.value_ptr_vec.push_back(std::make_shared<double>(0.0));
        }
        hdl_gpio_sensor_states_.push_back(temp_sensor);
      }
    }
    is_set_hdl_ = true;
  }
  for (auto it : hdl_gpio_controller_states_) {
    if (dxl_comm_->SetDxlReadItems(
        it.comm_id, it.id, it.interface_name_vec,
        it.value_ptr_vec) != DxlError::OK)
    {
      return false;
    }
  }
  for (auto it : hdl_trans_states_) {
    if (dxl_comm_->SetDxlReadItems(
        it.comm_id, it.id, it.interface_name_vec,
        it.value_ptr_vec) != DxlError::OK)
    {
      return false;
    }
  }
  for (auto it : hdl_gpio_sensor_states_) {
    if (dxl_comm_->SetDxlReadItems(
        it.comm_id, it.id, it.interface_name_vec,
        it.value_ptr_vec) != DxlError::OK)
    {
      return false;
    }
  }
  if (dxl_comm_->SetMultiDxlRead() != DxlError::OK) {
    return false;
  }
  return true;
}

bool DynamixelHardware::InitDxlWriteItems()
{
  RCLCPP_INFO_STREAM(logger_, "$$$$$ Init Dxl Write Items");
  is_set_hdl_ = false;

  if (!is_set_hdl_) {
    hdl_trans_commands_.clear();
    hdl_gpio_controller_commands_.clear();
    for (const hardware_interface::ComponentInfo & gpio : info_.gpios) {
      if (gpio.parameters.at("type") == "dxl") {
        uint8_t id = static_cast<uint8_t>(stoi(gpio.parameters.at("ID")));
        HandlerVarType temp_write;
        temp_write.id = id;
        temp_write.comm_id = id;
        temp_write.name = gpio.name;

        for (auto it : gpio.command_interfaces) {
          // if (it.name != "Goal Position" &&
          //   it.name != "Goal Velocity" &&
          //   it.name != "Goal Current")
          // {
          //   continue;
          // }
          temp_write.interface_name_vec.push_back(it.name);
          temp_write.value_ptr_vec.push_back(std::make_shared<double>(0.0));
        }
        hdl_trans_commands_.push_back(temp_write);
      } else if (gpio.parameters.at("type") == "controller") {
        uint8_t id = static_cast<uint8_t>(stoi(gpio.parameters.at("ID")));
        HandlerVarType temp_controller;
        temp_controller.id = id;
        temp_controller.comm_id = id;
        temp_controller.name = gpio.name;

        for (auto it : gpio.command_interfaces) {
          temp_controller.interface_name_vec.push_back(it.name);
          temp_controller.value_ptr_vec.push_back(std::make_shared<double>(0.0));
        }
        hdl_gpio_controller_commands_.push_back(temp_controller);
      } else if (gpio.parameters.at("type") == "virtual_dxl") {
        uint8_t id = static_cast<uint8_t>(stoi(gpio.parameters.at("ID")));
        uint8_t comm_id = static_cast<uint8_t>(stoi(gpio.parameters.at("comm_id")));
        HandlerVarType temp_write;
        temp_write.id = id;
        temp_write.comm_id = comm_id;
        temp_write.name = gpio.name;

        for (auto it : gpio.command_interfaces) {
          // if (it.name != "Goal Position" &&
          //   it.name != "Goal Velocity" &&
          //   it.name != "Goal Current")
          // {
          //   continue;
          // }
          temp_write.interface_name_vec.push_back(it.name);
          temp_write.value_ptr_vec.push_back(std::make_shared<double>(0.0));
        }
        hdl_trans_commands_.push_back(temp_write);
      }
    }
    is_set_hdl_ = true;
  }

  for (auto it : hdl_trans_commands_) {
    if (dxl_comm_->SetDxlWriteItems(
        it.comm_id, it.id, it.interface_name_vec,
        it.value_ptr_vec) != DxlError::OK)
    {
      return false;
    }
  }

  for (auto it : hdl_gpio_controller_commands_) {
    if (dxl_comm_->SetDxlWriteItems(
        it.comm_id, it.id, it.interface_name_vec,
        it.value_ptr_vec) != DxlError::OK)
    {
      return false;
    }
  }
  if (dxl_comm_->SetMultiDxlWrite() != DxlError::OK) {
    return false;
  }

  return true;
}

void DynamixelHardware::ReadSensorData(const HandlerVarType & sensor)
{
  for (auto item : sensor.interface_name_vec) {
    for (size_t i = 0; i < hdl_sensor_states_.size(); i++) {
      for (size_t j = 0; j < hdl_sensor_states_.at(i).interface_name_vec.size(); j++) {
        if (hdl_sensor_states_.at(i).name == sensor.name &&
          hdl_sensor_states_.at(i).interface_name_vec.at(j) == item)
        {
          *hdl_sensor_states_.at(i).value_ptr_vec.at(j) = *sensor.value_ptr_vec.at(j);
        }
      }
    }
  }
}

bool DynamixelHardware::SetMatrix()
{
  std::string str;
  std::vector<double> d_vec;

  // resize storage (number_of_transmissions x number_of_joint)
  transmission_to_joint_matrix_.assign(
    num_of_joints_, std::vector<double>(num_of_transmissions_, 0.0));

  d_vec.clear();
  if (info_.hardware_parameters.find("transmission_to_joint_matrix") ==
    info_.hardware_parameters.end())
  {
    RCLCPP_WARN_STREAM(
      logger_,
      "Parameter 'transmission_to_joint_matrix' not provided. Using identity matrix by default.");
    for (size_t i = 0; i < num_of_joints_; i++) {
      for (size_t j = 0; j < num_of_transmissions_; j++) {
        transmission_to_joint_matrix_[i][j] = (i == j) ? 1.0 : 0.0;
      }
    }
  } else {
    try {
      std::stringstream ss_tj(info_.hardware_parameters["transmission_to_joint_matrix"]);
      while (std::getline(ss_tj, str, ',')) {
        d_vec.push_back(stod(str));
      }
    } catch (const std::exception & e) {
      RCLCPP_ERROR_STREAM(
        logger_,
        "Failed to parse 'transmission_to_joint_matrix': " << e.what());
      return false;
    }
    const size_t expected_tj = num_of_joints_ * num_of_transmissions_;
    if (d_vec.size() != expected_tj) {
      RCLCPP_ERROR_STREAM(
        logger_,
        "Parameter 'transmission_to_joint_matrix' has " << d_vec.size()
                                                        << " elements, expected " << expected_tj);
      return false;
    }
    for (size_t i = 0; i < num_of_joints_; i++) {
      for (size_t j = 0; j < num_of_transmissions_; j++) {
        transmission_to_joint_matrix_[i][j] = d_vec.at(i * num_of_transmissions_ + j);
      }
    }
  }

  std::ostringstream transmission_to_joint_matrix_output;
  transmission_to_joint_matrix_output << "transmission_to_joint_matrix_\n";
  for (size_t i = 0; i < num_of_joints_; i++) {
    for (size_t j = 0; j < num_of_transmissions_; j++) {
      transmission_to_joint_matrix_output << "[" << i << "][" << j << "] "
                                          << transmission_to_joint_matrix_[i][j] << ", ";
    }
    transmission_to_joint_matrix_output << "\n";
  }
  RCLCPP_DEBUG_STREAM(logger_, transmission_to_joint_matrix_output.str());

  joint_to_transmission_matrix_.assign(
    num_of_transmissions_, std::vector<double>(num_of_joints_, 0.0));

  d_vec.clear();
  if (info_.hardware_parameters.find("joint_to_transmission_matrix") ==
    info_.hardware_parameters.end())
  {
    RCLCPP_WARN_STREAM(
      logger_,
      "Parameter 'joint_to_transmission_matrix' not provided. Using identity matrix by default.");
    for (size_t i = 0; i < num_of_transmissions_; i++) {
      for (size_t j = 0; j < num_of_joints_; j++) {
        joint_to_transmission_matrix_[i][j] = (i == j) ? 1.0 : 0.0;
      }
    }
  } else {
    try {
      std::stringstream ss_jt(info_.hardware_parameters["joint_to_transmission_matrix"]);
      while (std::getline(ss_jt, str, ',')) {
        d_vec.push_back(stod(str));
      }
    } catch (const std::exception & e) {
      RCLCPP_ERROR_STREAM(
        logger_,
        "Failed to parse 'joint_to_transmission_matrix': " << e.what());
      return false;
    }
    const size_t expected_jt = num_of_transmissions_ * num_of_joints_;
    if (d_vec.size() != expected_jt) {
      RCLCPP_ERROR_STREAM(
        logger_,
        "Parameter 'joint_to_transmission_matrix' has " << d_vec.size()
                                                        << " elements, expected " << expected_jt);
      return false;
    }
    for (size_t i = 0; i < num_of_transmissions_; i++) {
      for (size_t j = 0; j < num_of_joints_; j++) {
        joint_to_transmission_matrix_[i][j] = d_vec.at(i * num_of_joints_ + j);
      }
    }
  }


  std::ostringstream joint_to_transmission_matrix_output;
  joint_to_transmission_matrix_output << "joint_to_transmission_matrix_\n";
  for (size_t i = 0; i < num_of_transmissions_; i++) {
    for (size_t j = 0; j < num_of_joints_; j++) {
      joint_to_transmission_matrix_output << "[" << i << "][" << j << "] "
                                          << joint_to_transmission_matrix_[i][j] << ", ";
    }
    joint_to_transmission_matrix_output << "\n";
  }
  RCLCPP_DEBUG_STREAM(logger_, joint_to_transmission_matrix_output.str());

  return true;
}

void DynamixelHardware::MapInterfaces(
  size_t outer_size,
  size_t inner_size,
  std::vector<HandlerVarType> & outer_handlers,
  std::vector<HandlerVarType> & inner_handlers,
  const std::vector<std::vector<double>> & matrix,
  const std::unordered_map<std::string, std::vector<std::string>> & iface_map,
  const std::string & conversion_iface,
  const std::string & conversion_name,
  std::function<double(double)> conversion)
{
  for (size_t i = 0; i < outer_size; ++i) {
    for (size_t k = 0; k < outer_handlers.at(i).interface_name_vec.size(); ++k) {
      double value = 0.0;
      const std::string & outer_iface = outer_handlers.at(i).interface_name_vec.at(k);
      auto map_it = iface_map.find(outer_iface);
      if (map_it == iface_map.end()) {
        std::ostringstream oss;
        oss << "No mapping found for '" << outer_handlers.at(i).name
            << "', interface '" << outer_iface
            << "'. Skipping. Available mapping keys: [";
        size_t key_count = 0;
        for (const auto & pair : iface_map) {
          oss << pair.first;
          if (++key_count < iface_map.size()) {oss << ", ";}
        }
        oss << "]";
        RCLCPP_DEBUG_STREAM(logger_, oss.str());
        continue;
      }
      const std::vector<std::string> & mapped_ifaces = map_it->second;
      for (size_t j = 0; j < inner_size; ++j) {
        for (const auto & mapped_iface : mapped_ifaces) {
          auto it = std::find(
            inner_handlers.at(j).interface_name_vec.begin(),
            inner_handlers.at(j).interface_name_vec.end(),
            mapped_iface);
          if (it != inner_handlers.at(j).interface_name_vec.end()) {
            size_t idx = std::distance(inner_handlers.at(j).interface_name_vec.begin(), it);
            value += matrix[i][j] * (*inner_handlers.at(j).value_ptr_vec.at(idx));
            break;
          }
        }
      }
      if (!conversion_iface.empty() && !conversion_name.empty() &&
        outer_iface == conversion_iface &&
        outer_handlers.at(i).name == conversion_name &&
        conversion)
      {
        value = conversion(value);
      }
      *outer_handlers.at(i).value_ptr_vec.at(k) = value;
    }
  }
}

void DynamixelHardware::CalcTransmissionToJoint()
{
  std::function<double(double)> conv = use_revolute_to_prismatic_ ?
    std::function<double(double)>(
    std::bind(&DynamixelHardware::revoluteToPrismatic, this, std::placeholders::_1)) :
    std::function<double(double)>();
  this->MapInterfaces(
    num_of_joints_,
    num_of_transmissions_,
    hdl_joint_states_,
    hdl_trans_states_,
    transmission_to_joint_matrix_,
    dynamixel_hardware_interface::ros2_to_dxl_state_map,
    hardware_interface::HW_IF_POSITION,
    conversion_joint_name_,
    conv
  );
}

void DynamixelHardware::CalcJointToTransmission()
{
  std::function<double(double)> conv = use_revolute_to_prismatic_ ?
    std::function<double(double)>(
    std::bind(&DynamixelHardware::prismaticToRevolute, this, std::placeholders::_1)) :
    std::function<double(double)>();
  this->MapInterfaces(
    num_of_transmissions_,
    num_of_joints_,
    hdl_trans_commands_,
    hdl_joint_commands_,
    joint_to_transmission_matrix_,
    dynamixel_hardware_interface::dxl_to_ros2_cmd_map,
    "Goal Position",
    conversion_dxl_name_,
    conv
  );
}

void DynamixelHardware::SyncJointCommandWithStates()
{
  for (auto & it_states : hdl_joint_states_) {
    for (auto & it_commands : hdl_joint_commands_) {
      if (it_states.name == it_commands.name) {
        std::string pos_cmd_name = hardware_interface::HW_IF_POSITION;
        // Find index in command interfaces
        auto cmd_it = std::find(
          it_commands.interface_name_vec.begin(),
          it_commands.interface_name_vec.end(),
          pos_cmd_name);
        if (cmd_it == it_commands.interface_name_vec.end()) {
          // State-only joints (for example a passive gripper joint) are valid
          // and intentionally have no position command to synchronize.
          RCLCPP_DEBUG_STREAM(
            logger_,
            "No position interface found in command interfaces for joint '" <<
              it_commands.name << "'. Skipping sync!");
          continue;
        }
        size_t cmd_idx = std::distance(it_commands.interface_name_vec.begin(), cmd_it);
        // Find index in state interfaces
        auto state_it = std::find(
          it_states.interface_name_vec.begin(),
          it_states.interface_name_vec.end(),
          pos_cmd_name);
        if (state_it == it_states.interface_name_vec.end()) {
          RCLCPP_WARN_STREAM(
            logger_,
            "No position interface found in state interfaces for joint '" <<
              it_states.name << "'. Skipping sync!");
          continue;
        }
        size_t state_idx = std::distance(it_states.interface_name_vec.begin(), state_it);
        // Sync the value
        *it_commands.value_ptr_vec.at(cmd_idx) = *it_states.value_ptr_vec.at(state_idx);
        if (use_async_) {
          async_commands_.synchronizeCommand(it_commands.value_ptr_vec.at(cmd_idx));
        }
        RCLCPP_INFO_STREAM(
          logger_, "Sync joint state to command (joint: " << it_states.name << ", " <<
            it_commands.interface_name_vec.at(cmd_idx).c_str() << ", " <<
            *it_commands.value_ptr_vec.at(cmd_idx) << " <- " <<
            it_states.interface_name_vec.at(state_idx).c_str() << ", " <<
            *it_states.value_ptr_vec.at(state_idx));
      }
    }
  }
}

void DynamixelHardware::ChangeDxlTorqueState()
{
  if (torque_enabled_comm_id_id_.size() == 0) {
    return;
  }

  if (dxl_torque_status_ == REQUESTED_TO_ENABLE) {
    RCLCPP_WARN_STREAM(logger_, "Requested to enable torque, Enabling torque for all Dynamixels");
    dxl_comm_->DynamixelEnable(torque_enabled_comm_id_id_);
    SyncJointCommandWithStates();
  } else if (dxl_torque_status_ == REQUESTED_TO_DISABLE) {
    RCLCPP_WARN_STREAM(logger_, "Requested to disable torque, Disabling torque for all Dynamixels");
    dxl_comm_->DynamixelDisable(torque_enabled_comm_id_id_);
    SyncJointCommandWithStates();
  }

  // Aggregate across all devices; if any OFF, report DISABLED
  auto torque_state_map = dxl_comm_->GetDxlTorqueState();
  for (const auto & single_torque_state : torque_state_map) {
    if (single_torque_state.second == TORQUE_OFF) {
      dxl_torque_status_ = TORQUE_DISABLED;
      return;
    }
  }
  dxl_torque_status_ = TORQUE_ENABLED;
}

void DynamixelHardware::get_dxl_data_srv_callback(
  const std::shared_ptr<dynamixel_interfaces::srv::GetDataFromDxl::Request> request,
  std::shared_ptr<dynamixel_interfaces::srv::GetDataFromDxl::Response> response)
{
  uint8_t id = static_cast<uint8_t>(request->id);
  std::string name = request->item_name;

  if (dxl_comm_->InsertReadItemBuf(id, name) != DxlError::OK) {
    RCLCPP_ERROR_STREAM(logger_, "get_dxl_data_srv_callback InsertReadItemBuf");

    response->result = false;
    return;
  }
  double timeout_sec = request->timeout_sec;
  if (timeout_sec == 0.0) {
    timeout_sec = 1.0;
  }
  rclcpp::Time t_start = rclcpp::Clock().now();
  while (dxl_comm_->CheckReadItemBuf(id, name) == false) {
    if ((rclcpp::Clock().now() - t_start).seconds() > timeout_sec) {
      RCLCPP_ERROR_STREAM(
        logger_,
        "get_dxl_data_srv_callback Timeout : " << (rclcpp::Clock().now() - t_start).seconds() );
      response->result = false;
      return;
    }
  }

  response->item_data = dxl_comm_->GetReadItemDataBuf(id, name);
  response->result = true;
}

void DynamixelHardware::set_dxl_data_srv_callback(
  const std::shared_ptr<dynamixel_interfaces::srv::SetDataToDxl::Request> request,
  std::shared_ptr<dynamixel_interfaces::srv::SetDataToDxl::Response> response)
{
  uint8_t dxl_id = static_cast<uint8_t>(request->id);
  uint32_t dxl_data = static_cast<uint32_t>(request->item_data);
  if (dxl_comm_->InsertWriteItemBuf(dxl_id, request->item_name, dxl_data) == DxlError::OK) {
    response->result = true;
  } else {
    response->result = false;
  }
}

void DynamixelHardware::reboot_dxl_srv_callback(
  [[maybe_unused]] const std::shared_ptr<dynamixel_interfaces::srv::RebootDxl::Request> request,
  std::shared_ptr<dynamixel_interfaces::srv::RebootDxl::Response> response)
{
  if (CommReset()) {
    response->result = true;
    RCLCPP_INFO_STREAM(logger_, "[reboot_dxl_srv_callback] SUCCESS");
  } else {
    response->result = false;
    RCLCPP_INFO_STREAM(logger_, "[reboot_dxl_srv_callback] FAIL");
  }
}

void DynamixelHardware::set_dxl_torque_srv_callback(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  if (request->data) {
    if (dxl_torque_status_ == TORQUE_ENABLED) {
      response->success = true;
      response->message = "Already enabled.";
      RCLCPP_INFO_STREAM(logger_, "Requested to enable torque, but already enabled.");
      return;
    } else {
      dxl_torque_status_ = REQUESTED_TO_ENABLE;
    }
  } else {
    if (dxl_torque_status_ == TORQUE_DISABLED) {
      response->success = true;
      response->message = "Already disabled.";
      RCLCPP_INFO_STREAM(logger_, "Requested to disable torque, but already disabled.");
      return;
    } else {
      dxl_torque_status_ = REQUESTED_TO_DISABLE;
    }
  }
  response->success = true;
  response->message = "Success to write request.";

  // auto start = std::chrono::steady_clock::now();
  // while (std::chrono::steady_clock::now() - start < std::chrono::seconds(1)) {
  //   if (dxl_torque_status_ == TORQUE_ENABLED) {
  //     if (request->data) {
  //       response->success = true;
  //       response->message = "Success to enable.";
  //     } else {
  //       response->success = false;
  //       response->message = "Fail to enable.";
  //     }
  //     return;
  //   } else if (dxl_torque_status_ == TORQUE_DISABLED) {
  //     if (!request->data) {
  //       response->success = true;
  //       response->message = "Success to disable.";
  //     } else {
  //       response->success = false;
  //       response->message = "Fail to disable.";
  //     }
  //     return;
  //   }
  //   // std::this_thread::sleep_for(std::chrono::milliseconds(50));
  // }
  // response->success = false;
  // response->message = "Fail to write requeset. main thread is not running.";
}

void DynamixelHardware::initRevoluteToPrismaticParam()
{
  if (info_.hardware_parameters.find("revolute_to_prismatic_dxl") !=
    info_.hardware_parameters.end())
  {
    conversion_dxl_name_ = info_.hardware_parameters.at("revolute_to_prismatic_dxl");
  }

  if (info_.hardware_parameters.find("revolute_to_prismatic_joint") !=
    info_.hardware_parameters.end())
  {
    conversion_joint_name_ = info_.hardware_parameters.at("revolute_to_prismatic_joint");
  }

  if (info_.hardware_parameters.find("prismatic_min") != info_.hardware_parameters.end()) {
    prismatic_min_ = std::stod(info_.hardware_parameters.at("prismatic_min"));
  }

  if (info_.hardware_parameters.find("prismatic_max") != info_.hardware_parameters.end()) {
    prismatic_max_ = std::stod(info_.hardware_parameters.at("prismatic_max"));
  }

  if (info_.hardware_parameters.find("revolute_min") != info_.hardware_parameters.end()) {
    revolute_min_ = std::stod(info_.hardware_parameters.at("revolute_min"));
  }

  if (info_.hardware_parameters.find("revolute_max") != info_.hardware_parameters.end()) {
    revolute_max_ = std::stod(info_.hardware_parameters.at("revolute_max"));
  }

  conversion_slope_ = (prismatic_max_ - prismatic_min_) / (revolute_max_ - revolute_min_);
  conversion_intercept_ = prismatic_min_ - conversion_slope_ * revolute_min_;
}

double DynamixelHardware::revoluteToPrismatic(double revolute_value)
{
  return conversion_slope_ * revolute_value + conversion_intercept_;
}

double DynamixelHardware::prismaticToRevolute(double prismatic_value)
{
  return (prismatic_value - conversion_intercept_) / conversion_slope_;
}

std::string DynamixelHardware::getErrorSummary(uint8_t id) const
{
  std::ostringstream summary;
  summary << "Dynamixel ID " << static_cast<int>(id) << " Error Summary:\n";

  // Check hardware error status
  auto hw_err_it = dxl_hw_err_.find(id);
  if (hw_err_it != dxl_hw_err_.end() && hw_err_it->second != 0) {
    uint8_t hw_error_status = static_cast<uint8_t>(hw_err_it->second);
    summary << "  Hardware Error Status: 0x" << std::hex << static_cast<int>(hw_error_status)
            << std::dec << " (" << static_cast<int>(hw_error_status) << ")\n";

    for (int bit = 0; bit < 8; ++bit) {
      if (hw_error_status & (1 << bit)) {
        const HardwareErrorStatusBitInfo * bit_info = get_hardware_error_status_bit_info(bit);
        if (bit_info) {
          summary << "    Bit " << bit << ": " << bit_info->label
                  << " - " << bit_info->description << "\n";
        } else {
          summary << "    Bit " << bit << ": Unknown Error\n";
        }
      }
    }
  }

  // Check error code
  auto error_code_it = dxl_error_code_.find(id);
  if (error_code_it != dxl_error_code_.end() && error_code_it->second != 0x00) {
    uint8_t error_code = static_cast<uint8_t>(error_code_it->second);
    summary << "  Error Code: 0x" << std::hex << static_cast<int>(error_code)
            << std::dec << " (" << static_cast<int>(error_code) << ")\n";

    const ErrorCodeInfo * error_info = get_error_code_info(error_code);
    if (error_info) {
      summary << "    " << error_info->label << " - " << error_info->description << "\n";
    } else {
      summary << "    Unknown Error Code\n";
    }
  }

  // Check if no errors
  if ((hw_err_it == dxl_hw_err_.end() || hw_err_it->second == 0) &&
    (error_code_it == dxl_error_code_.end() || error_code_it->second == 0x00))
  {
    summary << "  No errors detected\n";
  }

  return summary.str();
}

std::string DynamixelHardware::getAllErrorSummaries() const
{
  std::ostringstream all_summaries;
  all_summaries << "=== All Dynamixel Error Summaries ===\n";

  // Get all unique IDs from both error maps
  std::set<uint8_t> all_ids;
  for (const auto & pair : dxl_hw_err_) {
    all_ids.insert(pair.first);
  }
  for (const auto & pair : dxl_error_code_) {
    all_ids.insert(pair.first);
  }

  if (all_ids.empty()) {
    all_summaries << "No Dynamixel IDs found in error maps.\n";
  } else {
    for (uint8_t id : all_ids) {
      all_summaries << getErrorSummary(id) << "\n";
    }
  }

  all_summaries << "=====================================\n";
  return all_summaries.str();
}

void DynamixelHardware::applyMultiTurnOffsets()
{
  for (auto & state : hdl_trans_states_) {
    for (size_t i = 0; i < state.interface_name_vec.size(); ++i) {
      if (state.interface_name_vec[i] == "Present Position") {
        *(state.value_ptr_vec[i]) += multi_turn_offsets_[state.id];
      }
    }
  }
}

void DynamixelHardware::removeMultiTurnOffsets()
{
  for (auto & cmd : hdl_trans_commands_) {
    for (size_t i = 0; i < cmd.interface_name_vec.size(); ++i) {
      if (cmd.interface_name_vec[i] == "Goal Position") {
        *(cmd.value_ptr_vec[i]) -= multi_turn_offsets_[cmd.id];
      }
    }
  }
}

void DynamixelHardware::restoreMultiTurnOffsets()
{
  for (auto & cmd : hdl_trans_commands_) {
    for (size_t i = 0; i < cmd.interface_name_vec.size(); ++i) {
      if (cmd.interface_name_vec[i] == "Goal Position") {
        *(cmd.value_ptr_vec[i]) += multi_turn_offsets_[cmd.id];
      }
    }
  }
}

}  // namespace dynamixel_hardware_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  dynamixel_hardware_interface::DynamixelHardware,
  hardware_interface::SystemInterface
)
