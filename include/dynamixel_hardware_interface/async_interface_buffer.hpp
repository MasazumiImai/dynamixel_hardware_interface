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

#ifndef DYNAMIXEL_HARDWARE_INTERFACE__ASYNC_INTERFACE_BUFFER_HPP_
#define DYNAMIXEL_HARDWARE_INTERFACE__ASYNC_INTERFACE_BUFFER_HPP_

#include <deque>
#include <memory>
#include <mutex>
#include <string>

namespace dynamixel_hardware_interface
{

// Public doubles belong to the control thread; hardware doubles belong to the I/O thread.
// Only the complete snapshot between them is shared. No lock is held during serial I/O.
class AsyncInterfaceBuffer
{
public:
  double * bind(const std::string & name, const std::shared_ptr<double> & hardware_value)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (auto & value : values_) {
      if (value.name == name) {
        value.hardware = hardware_value;
        return &value.control;
      }
    }
    values_.push_back({name, hardware_value, *hardware_value, *hardware_value, false});
    return &values_.back().control;
  }

  // Lifecycle initialization only: the worker and controllers must be stopped.
  void initialize()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (auto & value : values_) {
      value.control = value.snapshot = *value.hardware;
      value.synchronize = false;
    }
  }

  void captureState()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (auto & value : values_) {
      value.snapshot = *value.hardware;
    }
  }

  void readState()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (auto & value : values_) {
      value.control = value.snapshot;
    }
  }

  void writeCommands()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (auto & value : values_) {
      if (value.synchronize) {
        value.control = value.snapshot;
        value.synchronize = false;
      }
      value.snapshot = value.control;
    }
  }

  void applyCommands()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (auto & value : values_) {
      *value.hardware = value.snapshot;
    }
  }

  // Torque changes can request position synchronization from the I/O thread.
  void synchronizeCommand(const std::shared_ptr<double> & hardware_value)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (auto & value : values_) {
      if (value.hardware == hardware_value) {
        value.snapshot = *hardware_value;
        value.synchronize = true;
      }
    }
  }

  void readCommandUpdates()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (auto & value : values_) {
      if (value.synchronize) {
        value.control = value.snapshot;
        value.synchronize = false;
      }
    }
  }

private:
  struct Value
  {
    std::string name;
    std::shared_ptr<double> hardware;
    double control;
    double snapshot;
    bool synchronize;
  };

  // Appending another interface must not invalidate pointers already exported to ros2_control.
  std::deque<Value> values_;
  std::mutex mutex_;
};

}  // namespace dynamixel_hardware_interface

#endif  // DYNAMIXEL_HARDWARE_INTERFACE__ASYNC_INTERFACE_BUFFER_HPP_
