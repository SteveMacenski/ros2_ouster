// Copyright 2021, Steve Macenski
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

#ifndef ROS2_OUSTER__INTERFACES__CONFIGURATION_HPP_
#define ROS2_OUSTER__INTERFACES__CONFIGURATION_HPP_

#include <string>

namespace ros2_ouster
{
/**
 * @brief Configuration parameters to send to Ouster lidar
 */
struct Configuration
{
  std::string lidar_ip;
  std::string computer_ip;
  int imu_port;
  int lidar_port;
  std::string lidar_mode;
  std::string timestamp_mode;
  std::string metadata_filepath;
  std::string ethernet_device;
  std::string multipurpose_io_mode;
  std::string nmea_in_polarity;
  std::string nmea_baud_rate;
  std::string sync_pulse_in_polarity;
  bool phase_lock_enable;
  int phase_lock_offset;
};

}  // namespace ros2_ouster

#endif  // ROS2_OUSTER__INTERFACES__CONFIGURATION_HPP_
