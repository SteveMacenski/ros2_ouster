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

#include <string>
#include <fstream>
#include <sstream>
#include "ros2_ouster/client/client.h"
#include "ros2_ouster/exception.hpp"
#include "ros2_ouster/interfaces/metadata.hpp"
#include "ros2_ouster/sensor.hpp"

namespace sensor
{

Sensor::Sensor()
: SensorInterface() {}

Sensor::~Sensor()
{
  _ouster_client.reset();
}

void Sensor::reset(
  ros2_ouster::Configuration & config,
  rclcpp_lifecycle::LifecycleNode::SharedPtr node)
{
  _ouster_client.reset();
  configure(config, node);
}

void Sensor::configure(
  ros2_ouster::Configuration & config,
  rclcpp_lifecycle::LifecycleNode::SharedPtr node)
{
  RCLCPP_INFO(
    node->get_logger(), 
    "Configuring Ouster driver node.");

  // Check the validity of some of the retrieved parameters
  if (!ouster::sensor::lidar_mode_of_string(config.lidar_mode)) {
    throw ros2_ouster::OusterDriverException(
            "Invalid lidar mode: " + config.lidar_mode);
    exit(-1);
  }

  if (!ouster::sensor::timestamp_mode_of_string(config.timestamp_mode)) {
    throw ros2_ouster::OusterDriverException(
            "Invalid timestamp mode: " + config.timestamp_mode);
    exit(-1);
  }

  if (!ouster::sensor::multipurpose_io_mode_of_string(config.multipurpose_io_mode)) {
    throw ros2_ouster::OusterDriverException(
            "Invaild multipurpose io mode: " + config.multipurpose_io_mode);
    exit(-1);
  }

  if (!ouster::sensor::polarity_of_string(config.nmea_in_polarity)) {
    throw ros2_ouster::OusterDriverException(
            "Invaild nmea in polarity: " + config.nmea_in_polarity);
    exit(-1);
  }

  if (!ouster::sensor::polarity_of_string(config.sync_pulse_in_polarity)) {
    throw ros2_ouster::OusterDriverException(
            "Invaild sync pulse in polarity: " + config.sync_pulse_in_polarity);
    exit(-1);
  }

  if (!ouster::sensor::nmea_baud_rate_of_string(config.nmea_baud_rate)) {
    throw ros2_ouster::OusterDriverException(
            "Invaild nmea baud rate: " + config.nmea_baud_rate);
    exit(-1);
  }

  // Phase locking is only supported when time-synchronized from an external source (page 28. Ouster Software User Manual)
  if(config.phase_lock_enable){
    if (!(config.timestamp_mode == "TIME_FROM_PTP_1588" || config.timestamp_mode == "TIME_FROM_SYNC_PULSE_IN")) {
      throw ros2_ouster::OusterDriverException(
            "Phase Locking is only available when using external time-synchronization (TIME_FROM_PTP_1588 or TIME_FROM_SYNC_PULSE_IN):" + config.timestamp_mode);
      exit(-1);
    }
  }

  if(config.phase_lock_offset < 0 || config.phase_lock_enable > 360000) {
    throw ros2_ouster::OusterDriverException(
          "Phase lock offset must be between 0 and 360000 (0 to 360 degrees)" + std::to_string(config.phase_lock_offset));
      exit(-1);
  }

  // Report to the user whether automatic address detection is being used, and 
  // what the source / destination IPs are
  RCLCPP_INFO(
    node->get_logger(),
    "Connecting to sensor at %s.", config.lidar_ip.c_str());
  if (config.computer_ip == "") {
    RCLCPP_INFO(
      node->get_logger(),
      "Sending data from sensor to computer using automatic address detection");
  }  else {
    RCLCPP_INFO(
      node->get_logger(),
      "Sending data from sensor to %s.", config.computer_ip.c_str());
  }

  _ouster_client = ouster::sensor::init_client(
    config.lidar_ip,
    config.computer_ip,
    ouster::sensor::lidar_mode_of_string(config.lidar_mode),
    ouster::sensor::timestamp_mode_of_string(config.timestamp_mode),
    config.lidar_port,
    config.imu_port,
    60,
    ouster::sensor::multipurpose_io_mode_of_string(config.multipurpose_io_mode),
    ouster::sensor::polarity_of_string(config.nmea_in_polarity),
    ouster::sensor::polarity_of_string(config.sync_pulse_in_polarity),
    ouster::sensor::nmea_baud_rate_of_string(config.nmea_baud_rate),
    config.phase_lock_enable,
    config.phase_lock_offset);

  if (!_ouster_client) {
    throw ros2_ouster::OusterDriverException(
            std::string("Failed to create connection to lidar."));
  }

  setMetadata(config.lidar_port, config.imu_port, config.timestamp_mode);
}

ouster::sensor::client_state Sensor::get()
{
  const ouster::sensor::client_state state = ouster::sensor::poll_client(*_ouster_client);

  if (state == ouster::sensor::client_state::EXIT) {
    throw ros2_ouster::OusterDriverException(
            std::string(
              "Failed to get valid sensor data "
              "information from lidar, returned exit!"));
  } else if (state & ouster::sensor::client_state::CLIENT_ERROR) {
    throw ros2_ouster::OusterDriverException(
            std::string(
              "Failed to get valid sensor data "
              "information from lidar, returned error!"));
  }
  return state;
}

bool Sensor::readLidarPacket(const ouster::sensor::client_state & state, uint8_t * buf)
{
  if (state & ouster::sensor::client_state::LIDAR_DATA &&
    ouster::sensor::read_lidar_packet(
      *_ouster_client, buf,
      this->getPacketFormat()))
  {
    return true;
  }
  return false;
}

bool Sensor::readImuPacket(const ouster::sensor::client_state & state, uint8_t * buf)
{
  if (state & ouster::sensor::client_state::IMU_DATA &&
    ouster::sensor::read_imu_packet(
      *_ouster_client, buf,
      this->getPacketFormat()))
  {
    return true;
  }
  return false;
}

void Sensor::setMetadata(
  int lidar_port, int imu_port,
  const std::string & timestamp_mode)
{
  if (_ouster_client) {
    _metadata = ros2_ouster::Metadata(
      ouster::sensor::parse_metadata(
        ouster::sensor::get_metadata(*_ouster_client)),
      imu_port, lidar_port, timestamp_mode);
  }
  ros2_ouster::populate_missing_metadata_defaults(_metadata);
}

ros2_ouster::Metadata Sensor::getMetadata()
{
  return _metadata;
}

ouster::sensor::packet_format Sensor::getPacketFormat()
{
  return ouster::sensor::get_format(getMetadata());
}

}  // namespace sensor
