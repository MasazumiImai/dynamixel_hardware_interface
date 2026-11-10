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

// Regression: serial I/O and ros2_control must never share interface doubles.
#include <array>
#include <atomic>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <thread>

#include "dynamixel_hardware_interface/async_interface_buffer.hpp"

using dynamixel_hardware_interface::AsyncInterfaceBuffer;

void require(bool condition, const char * message)
{
  if (!condition) {
    throw std::runtime_error(message);
  }
}

int main()
{
  AsyncInterfaceBuffer states;
  auto hardware = std::make_shared<double>(0.5);
  double * exposed = states.bind("joint/position", hardware);
  require(exposed != hardware.get(), "The hardware double was exported directly");
  for (int i = 0; i < 4096; ++i) {
    states.bind("extra/" + std::to_string(i), std::make_shared<double>(0.0));
  }
  *hardware = 1.5;
  require(*exposed == 0.5, "Serial I/O changed an exposed state");
  states.captureState();
  require(*exposed == 0.5, "Publishing changed the control-side value");
  states.readState();
  require(*exposed == 1.5, "A complete state did not reach the controller");

  // Rebuilding communication handlers must preserve exported addresses.
  auto replacement = std::make_shared<double>(2.5);
  require(states.bind("joint/position", replacement) == exposed, "Rebinding moved an interface");
  states.captureState();
  states.readState();
  require(*exposed == 2.5, "A state still references the old communication handler");

  AsyncInterfaceBuffer commands;
  auto goal = std::make_shared<double>(3.0);
  double * command = commands.bind("joint/position", goal);
  commands.initialize();
  require(*command == 3.0, "Activation did not seed the command from hardware");
  *command = 4.0;
  require(*goal == 3.0, "The controller changed a hardware command directly");
  commands.writeCommands();
  require(*goal == 3.0, "Publishing bypassed the I/O thread");
  commands.applyCommands();
  require(*goal == 4.0, "A complete command did not reach the hardware");

  *goal = 5.0;
  commands.synchronizeCommand(goal);
  require(*command == 4.0, "An I/O-side position sync wrote into the control thread");
  commands.applyCommands();
  require(*goal == 5.0, "An old command overwrote a synchronized position");
  commands.readCommandUpdates();
  require(*command == 5.0, "Position synchronization did not reach the control thread");
  *command = 6.0;
  *goal = 7.0;
  commands.synchronizeCommand(goal);
  commands.writeCommands();
  commands.applyCommands();
  require(*command == 7.0 && *goal == 7.0, "A pending synchronization was overwritten");
  *command = 8.0;
  commands.writeCommands();
  commands.applyCommands();
  require(*goal == 8.0, "Commands did not resume after synchronization");

  constexpr size_t count = 64;
  AsyncInterfaceBuffer state_frames, command_frames;
  std::array<std::shared_ptr<double>, count> hardware_states, hardware_commands;
  std::array<double *, count> control_states, control_commands;
  for (size_t i = 0; i < count; ++i) {
    hardware_states[i] = std::make_shared<double>(0.0);
    hardware_commands[i] = std::make_shared<double>(0.0);
    control_states[i] = state_frames.bind(std::to_string(i), hardware_states[i]);
    control_commands[i] = command_frames.bind(std::to_string(i), hardware_commands[i]);
  }
  state_frames.initialize();
  command_frames.initialize();
  std::atomic<bool> begin{false};
  const auto run_io = [&]() {
      while (!begin.load(std::memory_order_acquire)) {
        std::this_thread::yield();
      }
      for (int frame = 1; frame <= 10000; ++frame) {
        for (size_t i = 0; i < count; ++i) {
          *hardware_states[i] = frame;
          if (i == count / 2) {
            std::this_thread::yield();
          }
        }
        state_frames.captureState();
        command_frames.applyCommands();
        const double expected = *hardware_commands[0];
        for (const auto & value : hardware_commands) {
          require(*value == expected, "The I/O thread consumed a partial command frame");
        }
      }
    };
  const auto run_control = [&]() {
      while (!begin.load(std::memory_order_acquire)) {
        std::this_thread::yield();
      }
      for (int frame = 1; frame <= 10000; ++frame) {
        state_frames.readState();
        const double expected = *control_states[0];
        for (double * value : control_states) {
          require(*value == expected, "The controller consumed a partial state frame");
        }
        for (size_t i = 0; i < count; ++i) {
          *control_commands[i] = -frame;
          if (i == count / 2) {
            std::this_thread::yield();
          }
        }
        command_frames.writeCommands();
      }
    };
  std::thread io(run_io);
  std::thread control(run_control);
  begin.store(true, std::memory_order_release);
  io.join();
  control.join();
  state_frames.readState();
  command_frames.applyCommands();
  for (size_t i = 0; i < count; ++i) {
    require(*control_states[i] == 10000, "The final state frame was lost");
    require(*hardware_commands[i] == -10000, "The final command frame was lost");
  }
  std::cout << "Async interface isolation and 10000 bidirectional frames passed.\n";
}
