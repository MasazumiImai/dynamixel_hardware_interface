// Copyright 2026 Space Robotics Lab -- Tohoku University
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

// Regressions: safe activation and receive timeouts independent of the worker period.
#include <algorithm>
#include <array>
#include <cstdint>
#include <deque>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "dynamixel_hardware_interface/dynamixel_hardware_interface.hpp"

namespace dynamixel_hardware_interface
{

void require(bool condition, const char * message)
{
  if (!condition) {throw std::runtime_error(message);}
}

// In-memory Protocol 2 port. The production SDK handles packets, CRCs and acknowledgements.
class MemoryPort : public dynamixel::PortHandler
{
public:
  std::array<std::array<uint8_t, 1024>, 4> registers{};
  std::vector<std::string> events;
  int fail_read_address = -1;
  bool fail_group_write = false;
  bool ignore_group_write = false;
  int fail_enable_id = -1;
  int ignore_enable_id = -1;
  int lose_enable_reply_id = -1;
  bool ignore_disable = false;
  bool fail_after_enable = false;
  bool hardware_alert = false;
  double group_reply_delay_ms = 0;
  bool drop_group_reply = false;
  bool truncate_group_reply = false;
  double elapsed_ms = 0;
  double timeout_ms = 0;

  MemoryPort() {is_using_ = false;}
  bool openPort() override {return true;}
  void closePort() override {}
  void clearPort() override {response_.clear(); received_bytes_ = 0;}
  void setPortName(const char *) override {}
  char * getPortName() override {return const_cast<char *>("memory");}
  bool setBaudRate(int) override {return true;}
  int getBaudRate() override {return 1000000;}
  int getBytesAvailable() override {return response_.size();}
  // Match the Linux SDK's packet-length timeout at 1 Mbps (16 ms latency allowance).
  void setPacketTimeout(uint16_t length) override {setPacketTimeout(length * 0.01 + 34);}
  void setPacketTimeout(double ms) override {timeout_ms = ms; elapsed_ms = 0;}
  bool isPacketTimeout() override {return elapsed_ms >= timeout_ms;}

  void set(uint8_t id, uint16_t address, uint32_t value, uint8_t size)
  {
    for (uint8_t i = 0; i < size; ++i) {registers.at(id).at(address + i) = value >> (8 * i);}
  }

  uint32_t get(uint8_t id, uint16_t address, uint8_t size) const
  {
    uint32_t value = 0;
    for (uint8_t i = 0; i < size; ++i) {
      value |= static_cast<uint32_t>(registers.at(id).at(address + i)) << (8 * i);
    }
    return value;
  }

  int readPort(uint8_t * packet, int length) override
  {
    elapsed_ms += 0.2;
    if (elapsed_ms < group_reply_delay_ms) {
      length = std::min(length, std::max(0, 16 - received_bytes_));
    }
    const int count = std::min(length, static_cast<int>(response_.size()));
    for (int i = 0; i < count; ++i) {
      packet[i] = response_.front();
      response_.pop_front();
    }
    received_bytes_ += count;
    return count;
  }

  int writePort(uint8_t * packet, int length) override
  {
    const uint8_t id = packet[4];
    const uint8_t instruction = packet[7];
    const uint16_t address = packet[8] | (packet[9] << 8);
    const bool group_read = instruction == 0x82 || instruction == 0x8a ||
      instruction == 0x92 || instruction == 0x9a;
    if (instruction == 2) {
      events.push_back("read:" + std::to_string(id) + ":" + std::to_string(address));
      if (address == fail_read_address ||
        (fail_after_enable && get(1, 64, 1) && address == 132))
      {
        return length;
      }
      const uint16_t size = packet[10] | (packet[11] << 8);
      std::vector<uint8_t> data;
      for (uint16_t i = 0; i < size; ++i) {data.push_back(readByte(id, address + i));}
      respond(id, data, hardware_alert ? 0x80 : 0);
    } else if (instruction == 3) {
      if (address == 64) {
        const bool enable = packet[10] != 0;
        events.push_back(std::string(enable ? "on:" : "off:") + std::to_string(id));
        if (enable && id == fail_enable_id) {
          respond(id, {}, 4);
          return length;
        }
        if ((enable && id == ignore_enable_id) || (!enable && ignore_disable)) {
          respond(id, {});
          return length;
        }
      }
      for (int i = 10; i < length - 2; ++i) {writeByte(id, address + i - 10, packet[i]);}
      if (address == 64 && packet[10] && id == lose_enable_reply_id) {return length;}
      respond(id, {});
    } else if (instruction == 0x83) {
      events.push_back("group_stop");
      if (fail_group_write) {return -1;}
      const uint16_t size = packet[10] | (packet[11] << 8);
      if (!ignore_group_write) {
        for (int i = 12; i < length - 2; i += size + 1) {
          for (uint16_t j = 0; j < size; ++j) {
            writeByte(packet[i], address + j, packet[i + 1 + j]);
          }
        }
      }
    } else if (group_read) {
      const bool sync = instruction == 0x82 || instruction == 0x8a;
      const bool fast = instruction == 0x8a || instruction == 0x9a;
      std::vector<uint8_t> combined;
      for (int i = sync ? 12 : 8; i < length - 2; i += sync ? 1 : 5) {
        const uint16_t start = sync ? address : packet[i + 1] | (packet[i + 2] << 8);
        const uint16_t size = sync ? packet[10] | (packet[11] << 8) :
          packet[i + 3] | (packet[i + 4] << 8);
        std::vector<uint8_t> data;
        for (uint16_t j = 0; j < size; ++j) {data.push_back(readByte(packet[i], start + j));}
        if (fast) {
          combined.push_back(0);
          combined.push_back(packet[i]);
          combined.insert(combined.end(), data.begin(), data.end());
          combined.insert(combined.end(), {0, 0});
        } else {
          respond(packet[i], data);
        }
      }
      if (fast) {
        // respond() supplies the first error byte and the final CRC of the aggregate packet.
        combined.erase(combined.begin());
        combined.resize(combined.size() - 2);
        respond(0xfe, combined);
      }
      if (drop_group_reply) {response_.clear();}
      if (truncate_group_reply && response_.size() > 16) {response_.resize(16);}
    } else {
      throw std::runtime_error("Unexpected SDK instruction");
    }
    return length;
  }

private:
  std::deque<uint8_t> response_;
  int received_bytes_ = 0;

  uint16_t resolve(uint8_t id, uint16_t address) const
  {
    if (address >= 224 && address < 252) {return get(id, 168 + 2 * (address - 224), 2);}
    if (address >= 634 && address < 662) {return get(id, 578 + 2 * (address - 634), 2);}
    return address;
  }
  uint8_t readByte(uint8_t id, uint16_t address) const
  {
    return registers.at(id).at(resolve(id, address));
  }
  void writeByte(uint8_t id, uint16_t address, uint8_t value)
  {
    registers.at(id).at(resolve(id, address)) = value;
  }
  void respond(uint8_t id, const std::vector<uint8_t> & data, uint8_t error = 0)
  {
    const uint16_t length = data.size() + 4;
    std::vector<uint8_t> packet{0xff, 0xff, 0xfd, 0, id,
      static_cast<uint8_t>(length), static_cast<uint8_t>(length >> 8), 0x55, error};
    packet.insert(packet.end(), data.begin(), data.end());
    uint16_t crc = 0;
    for (uint8_t byte : packet) {
      crc ^= static_cast<uint16_t>(byte) << 8;
      for (int bit = 0; bit < 8; ++bit) {
        crc = (crc & 0x8000) ? (crc << 1) ^ 0x8005 : crc << 1;
      }
    }
    packet.push_back(crc);
    packet.push_back(crc >> 8);
    response_.insert(response_.end(), packet.begin(), packet.end());
  }
};

struct StartupCheck
{
  static void checkReceiveTimeout(DynamixelHardware & hardware, MemoryPort & port)
  {
    auto & communication = *hardware.dxl_comm_;
    using Return = hardware_interface::return_type;
    const rclcpp::Time time(0);
    const auto period = rclcpp::Duration::from_seconds(0.00318605);
    hardware.err_timeout_ms_ = 500;
    // The worker has been joined by stop(); exercise its common read path without hardware.
    for (bool sync : {true, false}) {
      for (bool fast : {false, true}) {
        communication.read_type_ = sync ? SYNC : BULK;
        communication.use_fast_read_protocol_ = fast;
        communication.fast_read_permanent_ = false;
        communication.fast_read_fail_count_ = 0;
        if (sync) {
          require(
            communication.SetSyncReadHandler({1, 2, 3}) == DxlError::OK,
            "Sync read setup failed");
        } else {
          require(
            communication.SetBulkReadItemAndHandler() == DxlError::OK,
            "Bulk read setup failed");
        }
        // A valid reply arrives in two USB chunks, with the remainder at 4.5 ms.
        port.group_reply_delay_ms = 4.5;
        *hardware.hdl_trans_states_[0].value_ptr_vec[0] = -999;
        require(hardware.readCommunication(time, period) == Return::OK, "Delayed read failed");
        require(!hardware.is_read_in_error_, "Worker period truncated a valid delayed reply");
        const double position = *hardware.hdl_trans_states_[0].value_ptr_vec[0];
        require(position != -999, "Delayed read did not refresh feedback");
        require(port.elapsed_ms >= 4.5, "Delayed reply was not exercised");
        require(port.elapsed_ms < port.timeout_ms, "Read waited after receiving a complete reply");
        for (bool partial : {false, true}) {
          port.drop_group_reply = !partial;
          port.truncate_group_reply = partial;
          require(
            hardware.readCommunication(time, period) == Return::OK,
            "Transient read failure bypassed the error grace period");
          require(hardware.is_read_in_error_, "Missing/incomplete reply was accepted");
          require(
            *hardware.hdl_trans_states_[0].value_ptr_vec[0] == position,
            "Failed read overwrote the last feedback");
          require(
            port.elapsed_ms >= port.timeout_ms && port.elapsed_ms < 100,
            "Receive timeout was not bounded independently of the error grace period");
          require(
            hardware.readCommunication(time, rclcpp::Duration::from_seconds(0.5)) == Return::ERROR,
            "Persistent read failure bypassed the error timeout");
          port.drop_group_reply = false;
          port.truncate_group_reply = false;
          require(hardware.readCommunication(time, period) == Return::OK, "Read did not recover");
          require(!hardware.is_read_in_error_, "Successful read did not clear the error duration");
        }
        port.group_reply_delay_ms = 0;
      }
    }
    std::cout << "Receive timeout checks passed (normal/fast SyncRead and BulkRead).\n";
  }

  static void run(const char * models)
  {
    using Callback = hardware_interface::CallbackReturn;
    // Each failure is injected after setup, so it exercises activation rather than test setup.
    for (const std::string failure : {"none", "read", "write", "ignored_write", "readback", "mode",
        "enable", "ignored_enable", "lost_enable_reply", "disable", "after_enable", "nan",
        "missing_position", "missing_stop", "nan_command", "alert", "restart", "empty_read",
        "position_hold", "bad_position_hold"})
    {
      for (bool async : {false, true}) {
        DynamixelHardware hardware;
        hardware.dxl_comm_ = std::make_shared<Dynamixel>(models);
        auto & communication = *hardware.dxl_comm_;
        auto * port = new MemoryPort();
        communication.port_handler_ = port;
        communication.packet_handler_ = dynamixel::PacketHandler::getPacketHandler(2.0);
        communication.use_fast_read_protocol_ = false;
        const bool position_mode = failure == "position_hold" || failure == "bad_position_hold";
        hardware.use_async_ = async;
        hardware.err_timeout_ms_ = 0;
        hardware.num_of_joints_ = 3;
        hardware.num_of_transmissions_ = 3;
        hardware.transmission_to_joint_matrix_ = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
        hardware.joint_to_transmission_matrix_ = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
        for (uint8_t id : {1, 2, 3}) {
          require(
            communication.ReadDxlModelFile(id, id, 1020) == DxlError::OK,
            "Model load failed");
          port->set(id, 11, position_mode ? 4 : 1, 1);
          port->set(id, 132, 2048 + 100 * id, 4);
          // Seed an old motion command in both the device and retained command storage.
          port->set(id, 104, 50, 4);
          hardware.dxl_comm_id_id_.emplace_back(id, id);
          if (id != 3) {hardware.torque_enabled_comm_id_id_.emplace_back(id, id);}
          const std::string name = "joint" + std::to_string(id);
          HandlerVarType state{id, id, name, {"Present Position", "Present Velocity"},
            {std::make_shared<double>(0), std::make_shared<double>(0)}};
          HandlerVarType command{id, id, name, {}, {}};
          if (id != 3) {
            command.interface_name_vec = {position_mode ? "Goal Position" : "Goal Velocity"};
            command.value_ptr_vec = {std::make_shared<double>(3)};
          }
          require(
            communication.SetDxlReadItems(
              id, id, state.interface_name_vec,
              state.value_ptr_vec) == DxlError::OK, "Read setup failed");
          require(
            communication.SetDxlWriteItems(
              id, id, command.interface_name_vec,
              command.value_ptr_vec) == DxlError::OK, "Write setup failed");
          hardware.hdl_trans_states_.push_back(state);
          hardware.hdl_trans_commands_.push_back(command);
          hardware.hdl_joint_states_.push_back(
            {id, id, name, {"position", "velocity"},
              {std::make_shared<double>(0), std::make_shared<double>(0)}});
          hardware.hdl_joint_commands_.push_back(
            {id, id, name, {"position", "velocity"},
              {std::make_shared<double>(10), std::make_shared<double>(3)}});
          hardware_interface::ComponentInfo gpio;
          gpio.name = name;
          gpio.parameters = {{"ID", std::to_string(id)}};
          if (id != 3) {gpio.parameters["Operating Mode"] = position_mode ? "4" : "1";}
          hardware_interface::InterfaceInfo interface;
          interface.name = position_mode ? "Goal Position" : "Goal Velocity";
          if (id != 3) {gpio.command_interfaces.push_back(interface);}
          hardware.info_.gpios.push_back(gpio);
        }
        require(communication.SetMultiDxlRead() == DxlError::OK, "Grouped read setup failed");
        require(communication.SetMultiDxlWrite() == DxlError::OK, "Grouped write setup failed");
        port->events.clear();
        if (failure == "empty_read") {communication.read_data_list_.clear();}
        if (failure == "bad_position_hold") {hardware.joint_to_transmission_matrix_[0][0] = 0;}
        if (failure == "read") {port->fail_read_address = 132;}
        if (failure == "write") {port->fail_group_write = true;}
        if (failure == "ignored_write") {port->ignore_group_write = true;}
        if (failure == "readback") {port->fail_read_address = 104;}
        if (failure == "mode") {port->set(2, 11, 3, 1);}
        if (failure == "enable") {port->fail_enable_id = 2;}
        if (failure == "ignored_enable") {port->ignore_enable_id = 2;}
        if (failure == "lost_enable_reply") {port->lose_enable_reply_id = 2;}
        if (failure == "disable") {port->set(1, 64, 1, 1); port->ignore_disable = true;}
        if (failure == "after_enable") {port->fail_after_enable = true;}
        if (failure == "nan") {
          communication.OverrideUnitInfo(
            2, 2, "Present Position",
            std::numeric_limits<double>::quiet_NaN(), true, 0);
        }
        if (failure == "missing_position") {
          hardware.hdl_joint_states_[1].interface_name_vec[0] = "unmapped";
        }
        if (failure == "missing_stop") {hardware.info_.gpios[1].command_interfaces.clear();}
        if (failure == "nan_command") {
          hardware.joint_to_transmission_matrix_[0][0] = std::numeric_limits<double>::quiet_NaN();
        }
        if (failure == "alert") {port->hardware_alert = true;}
        if (failure == "restart") {
          require(hardware.start() == Callback::SUCCESS, "Initial activation failed");
          require(hardware.stop() == Callback::SUCCESS, "Initial deactivation failed");
          *hardware.hdl_joint_commands_[0].value_ptr_vec[1] = 3;
          port->set(1, 104, 50, 4);
          port->events.clear();
        }
        const auto result = hardware.start();
        if (failure == "none" || failure == "restart" || failure == "position_hold") {
          require(result == Callback::SUCCESS, "Healthy activation failed");
          require(hardware.stop() == Callback::SUCCESS, "Healthy deactivation failed");
          const auto on = std::find(port->events.begin(), port->events.end(), "on:1");
          require(on != port->events.end(), "Torque was never enabled");
          const auto stop = std::find(port->events.begin(), on, "group_stop");
          require(stop != on, "Torque enabled before stop transmission");
          for (uint8_t id : {1, 2}) {
            const auto readback = "read:" + std::to_string(id) + (position_mode ? ":116" : ":104");
            require(std::find(stop, on, readback) != on, "Torque enabled before command readback");
            if (position_mode) {
              require(
                std::abs(
                  static_cast<int64_t>(port->get(id, 116, 4)) -
                  port->get(id, 132, 4)) <= 1, "Position mode did not hold the initial position");
            } else {
              require(port->get(id, 104, 4) == 0, "Old device velocity survived activation");
            }
            require(
              *hardware.hdl_joint_commands_[id - 1].value_ptr_vec[1] == 0,
              "Old control velocity survived activation");
            require(
              *hardware.hdl_joint_states_[id - 1].value_ptr_vec[0] != 0,
              "Initial position was not read");
          }
        } else {
          require(result == Callback::ERROR, "Unsafe activation reported success");
          require(!hardware.comm_thread_.joinable(), "Worker started after failed activation");
          if (async) {
            require(
              hardware.async_read_status_.load() == hardware_interface::return_type::ERROR &&
              hardware.async_write_status_.load() == hardware_interface::return_type::ERROR,
              "Failed activation published healthy I/O status");
          }
          if (failure != "disable") {
            require(
              port->get(1, 64, 1) == 0 && port->get(2, 64, 1) == 0,
              "Failed activation left torque enabled");
          }
          if (failure != "enable" && failure != "ignored_enable" &&
            failure != "lost_enable_reply" && failure != "after_enable")
          {
            require(
              std::find(port->events.begin(), port->events.end(), "on:1") == port->events.end(),
              "Torque enabled before the prerequisite checks passed");
          }
        }
        require(
          std::find(port->events.begin(), port->events.end(), "on:3") == port->events.end(),
          "A passive motor was enabled");
        if (failure == "none") {checkReceiveTimeout(hardware, *port);}
      }
    }
  }
};

}  // namespace dynamixel_hardware_interface

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    dynamixel_hardware_interface::require(argc == 2, "Pass the Dynamixel model directory");
    dynamixel_hardware_interface::StartupCheck::run(argv[1]);
    rclcpp::shutdown();
    std::cout << "Startup interlock checks passed (synchronous and asynchronous).\n";
    return 0;
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n';
    rclcpp::shutdown();
    return 1;
  }
}
