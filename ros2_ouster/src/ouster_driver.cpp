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

#include <chrono>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/qos.hpp"
#include "ros2_ouster/exception.hpp"
#include "ros2_ouster/interfaces/lifecycle_interface.hpp"
#include "ros2_ouster/interfaces/sensor_interface.hpp"
#include "ros2_ouster/ouster_driver.hpp"
#include "ros2_ouster/processors/processor_factories.hpp"
#include "ros2_ouster/client/types.h"
#include "ros2_ouster/sensor.hpp"
#include "ros2_ouster/sensor_tins.hpp"

namespace ros2_ouster
{

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;
using namespace std::chrono_literals;

OusterDriver::OusterDriver(
  std::unique_ptr<SensorInterface> sensor,
  const rclcpp::NodeOptions & options)
: LifecycleInterface("OusterDriver", options), _sensor{std::move(sensor)}
{
  // Declare parameters for configuring the _driver_
  this->declare_parameter("sensor_frame", std::string("laser_sensor_frame"));
  this->declare_parameter("laser_frame", std::string("laser_data_frame"));
  this->declare_parameter("imu_frame", std::string("imu_data_frame"));
  this->declare_parameter("use_system_default_qos", false);
  this->declare_parameter("proc_mask", std::string("IMG|PCL|IMU|SCAN"));

  // Declare parameters used across ALL _sensor_ implementations
  this->declare_parameter<std::string>("lidar_ip","10.5.5.96");
  this->declare_parameter<std::string>("computer_ip","10.5.5.1");
  this->declare_parameter("imu_port", 7503);
  this->declare_parameter("lidar_port", 7502);
  this->declare_parameter("lidar_mode", std::string("512x10"));
  this->declare_parameter("timestamp_mode", std::string("TIME_FROM_INTERNAL_OSC"));
  this->declare_parameter("multipurpose_io_mode", std::string("OFF"));
  this->declare_parameter("nmea_in_polarity", std::string("ACTIVE_HIGH"));
  this->declare_parameter("sync_pulse_in_polarity", std::string("ACTIVE_HIGH"));
  this->declare_parameter("nmea_baud_rate", std::string("BAUD_9600"));
  this->declare_parameter("phase_lock_enable", false);
  this->declare_parameter<int>("phase_lock_offset", 0);
}

OusterDriver::~OusterDriver() = default;

void OusterDriver::onConfigure()
{
  // Get parameters for configuring the _driver_
  _laser_sensor_frame = get_parameter("sensor_frame").as_string();
  _laser_data_frame = get_parameter("laser_frame").as_string();
  _imu_data_frame = get_parameter("imu_frame").as_string();
  _use_system_default_qos = get_parameter("use_system_default_qos").as_bool();
  _proc_mask = ros2_ouster::toProcMask(get_parameter("proc_mask").as_string());

  // Get parameters used across ALL _sensor_ implementations. Parameters unique
  // a specific Sensor implementation are "getted" in the configure() function
  // for that sensor.
  ros2_ouster::Configuration lidar_config;
  lidar_config.imu_port = this->get_parameter("imu_port").as_int();
  lidar_config.lidar_port = this->get_parameter("lidar_port").as_int();
  lidar_config.lidar_mode = this->get_parameter("lidar_mode").as_string();
  lidar_config.timestamp_mode = this->get_parameter("timestamp_mode").as_string();
  lidar_config.multipurpose_io_mode = this->get_parameter("multipurpose_io_mode").as_string();
  lidar_config.nmea_in_polarity = this->get_parameter("nmea_in_polarity").as_string();
  lidar_config.nmea_baud_rate = this->get_parameter("nmea_baud_rate").as_string();
  lidar_config.sync_pulse_in_polarity = this->get_parameter("sync_pulse_in_polarity").as_string();
  lidar_config.phase_lock_enable = this->get_parameter("phase_lock_enable").as_bool();
  lidar_config.phase_lock_offset = this->get_parameter("phase_lock_offset").as_int();

  // Deliberately retrieve the IP parameters in a try block without defaults, as
  // we cannot estimate a reasonable default IP address for the LiDAR/computer.
  try {
    lidar_config.lidar_ip = get_parameter("lidar_ip").as_string();
    lidar_config.computer_ip = get_parameter("computer_ip").as_string();
  } catch (...) {
    RCLCPP_FATAL(
      this->get_logger(),
      "Failed to get lidar or IMU IP address or "
      "hostname. An IP address for both are required!");
    exit(-1);
  }

  if (lidar_config.timestamp_mode == "TIME_FROM_ROS_RECEPTION") {
    RCLCPP_WARN(
      this->get_logger(),
      "Using TIME_FROM_ROS_RECEPTION to stamp data with ROS time on "
      "reception. This has unmodelled latency!");
    lidar_config.timestamp_mode = "TIME_FROM_INTERNAL_OSC";
    _use_ros_time = true;
  } else {
    _use_ros_time = false;
  }

  // Configure the driver and sensor
  try {
    _sensor->configure(lidar_config, shared_from_this());
  } catch (const OusterDriverException & e) {
    RCLCPP_FATAL(this->get_logger(), "Exception thrown: (%s)", e.what());
    exit(-1);
  }

  RCLCPP_INFO(
    this->get_logger(),
    "This driver is compatible with sensors running fw 2.2-2.4");

  _reset_srv = this->create_service<std_srvs::srv::Empty>(
    "~/reset", std::bind(&OusterDriver::resetService, this, _1, _2, _3));
  _metadata_srv = this->create_service<ouster_msgs::srv::GetMetadata>(
    "~/get_metadata", std::bind(&OusterDriver::getMetadata, this, _1, _2, _3));

  _full_rotation_accumulator = std::make_shared<sensor::FullRotationAccumulator>(
    _sensor->getMetadata(), _sensor->getPacketFormat());

  if (_use_system_default_qos) {
    RCLCPP_INFO(
      this->get_logger(), "Using system defaults QoS for sensor data");
    _data_processors = ros2_ouster::createProcessors(
      shared_from_this(), _sensor->getMetadata(), _imu_data_frame, _laser_data_frame,
      rclcpp::SystemDefaultsQoS(),
      _sensor->getPacketFormat(), _full_rotation_accumulator, _proc_mask);
  } else {
    _data_processors = ros2_ouster::createProcessors(
      shared_from_this(), _sensor->getMetadata(), _imu_data_frame, _laser_data_frame,
      rclcpp::SensorDataQoS(), _sensor->getPacketFormat(), _full_rotation_accumulator, _proc_mask);
  }

  _tf_b = std::make_unique<tf2_ros::StaticTransformBroadcaster>(
    shared_from_this());
  broadcastStaticTransforms(_sensor->getMetadata());
}

void OusterDriver::onActivate()
{
  DataProcessorMapIt it;
  for (it = _data_processors.begin(); it != _data_processors.end(); ++it) {
    it->second->onActivate();
  }

  _lidar_packet_buf = std::make_unique<RingBuffer>(
    _sensor->getPacketFormat().lidar_packet_size, 1024);
  _imu_packet_buf = std::make_unique<RingBuffer>(
    _sensor->getPacketFormat().imu_packet_size, 1024);

  _processing_active = true;
  _process_thread = std::thread(std::bind(&OusterDriver::processData, this));
  _recv_thread = std::thread(std::bind(&OusterDriver::receiveData, this));
}

void OusterDriver::onError()
{
}

void OusterDriver::onDeactivate()
{
  _processing_active = false;

  if (_recv_thread.joinable()) {
    _recv_thread.join();
  }

  _process_cond.notify_all();
  if (_process_thread.joinable()) {
    _process_thread.join();
  }

  DataProcessorMapIt it;
  for (it = _data_processors.begin(); it != _data_processors.end(); ++it) {
    it->second->onDeactivate();
  }
}

void OusterDriver::onCleanup()
{
  _data_processors.clear();
  _tf_b.reset();
  _reset_srv.reset();
  _metadata_srv.reset();
}

void OusterDriver::onShutdown()
{
  _tf_b.reset();

  DataProcessorMapIt it;
  _data_processors.clear();
}

void OusterDriver::broadcastStaticTransforms(
  const ouster::sensor::sensor_info & mdata)
{
  if (_tf_b) {
    std::vector<geometry_msgs::msg::TransformStamped> transforms;
    transforms.push_back(
      toMsg(
        mdata.imu_to_sensor_transform,
        _laser_sensor_frame, _imu_data_frame, this->now()));
    transforms.push_back(
      toMsg(
        mdata.lidar_to_sensor_transform,
        _laser_sensor_frame, _laser_data_frame, this->now()));
    _tf_b->sendTransform(transforms);
  }
}

void OusterDriver::processData() {
  std::pair<DataProcessorMapIt, DataProcessorMapIt> key_lidar_its =
    _data_processors.equal_range(ouster::sensor::client_state::LIDAR_DATA);
  std::pair<DataProcessorMapIt, DataProcessorMapIt> key_imu_its =
    _data_processors.equal_range(ouster::sensor::client_state::IMU_DATA);

  std::unique_lock<std::mutex> ringbuffer_guard(_ringbuffer_mutex);
  while (_processing_active) {
    // Wait for data in either the lidar or imu ringbuffer
    _process_cond.wait(
      ringbuffer_guard, [this] () {
        return (!_lidar_packet_buf->empty() ||
                !_imu_packet_buf->empty() ||
                !_processing_active);
    });
    ringbuffer_guard.unlock();

    uint64_t override_ts = this->_use_ros_time ? this->now().nanoseconds() : 0;

    // If we have data in the lidar buffer, process it
    if (!_lidar_packet_buf->empty() && _processing_active) {
      _full_rotation_accumulator->accumulate(_lidar_packet_buf->head(), override_ts);
      for (DataProcessorMapIt it = key_lidar_its.first; it != key_lidar_its.second; it++) {
        it->second->process(_lidar_packet_buf->head(), override_ts);
      }
      _lidar_packet_buf->pop();
    }

    // If we have data in the imu buffer, process it
    if (!_imu_packet_buf->empty() && _processing_active) {
      for (DataProcessorMapIt it = key_imu_its.first; it != key_imu_its.second; it++) {
        it->second->process(_imu_packet_buf->head(), override_ts);
      }
      _imu_packet_buf->pop();
    }

    ringbuffer_guard.lock();
  }
}

void OusterDriver::receiveData()
{
  while (_processing_active) {
    try {
      // Receive raw sensor data from the network.
      // This blocks for some time until either data is received or timeout
      ouster::sensor::client_state state = _sensor->get();
      bool got_lidar = _sensor->readLidarPacket(state, _lidar_packet_buf->tail());
      bool got_imu = _sensor->readImuPacket(state, _imu_packet_buf->tail());

      // If we got some data, push to ringbuffer and signal processing thread
      if (got_lidar || got_imu) {
        if (got_lidar) {
          // If the ringbuffer is full, this means the processing thread is running too slow to
          // process all frames. Therefore, we push to it anyway, discarding all (old) data
          // in the buffer and emit a warning.
          if (_lidar_packet_buf->full()) {
            RCLCPP_WARN(this->get_logger(), "Lidar buffer overrun!");
          }
          _lidar_packet_buf->push();
        }

        if (got_imu) {
          if (_imu_packet_buf->full()) {
            RCLCPP_WARN(this->get_logger(), "IMU buffer overrun!");
          }
          _imu_packet_buf->push();
        }
        _process_cond.notify_all();
      }

      RCLCPP_DEBUG(
        this->get_logger(),
        "Retrieved packet with state: %s",
        ros2_ouster::toString(state).c_str());
    } catch (const OusterDriverException & e) {
      RCLCPP_WARN(
        this->get_logger(),
        "Failed to process packet with exception %s.", e.what());
    }
  }
}

void OusterDriver::resetService(
  const std::shared_ptr<rmw_request_id_t>/*request_header*/,
  const std::shared_ptr<std_srvs::srv::Empty::Request> request,
  std::shared_ptr<std_srvs::srv::Empty::Response> response)
{
  if (!this->isActive()) {
    return;
  }

  ros2_ouster::Configuration lidar_config;
  lidar_config.lidar_ip = get_parameter("lidar_ip").as_string();
  lidar_config.computer_ip = get_parameter("computer_ip").as_string();
  lidar_config.imu_port = get_parameter("imu_port").as_int();
  lidar_config.lidar_port = get_parameter("lidar_port").as_int();
  lidar_config.lidar_mode = get_parameter("lidar_mode").as_string();
  lidar_config.timestamp_mode = get_parameter("timestamp_mode").as_string();
  _sensor->reset(lidar_config, shared_from_this());
}

void OusterDriver::getMetadata(
  const std::shared_ptr<rmw_request_id_t>/*request_header*/,
  const std::shared_ptr<ouster_msgs::srv::GetMetadata::Request> request,
  std::shared_ptr<ouster_msgs::srv::GetMetadata::Response> response)
{
  if (!this->isActive()) {
    return;
  }
  response->metadata = toMsg(_sensor->getMetadata());

  // Save the metadata to file ONLY if the user specifies a filepath
  if (request->metadata_filepath != "") {
    std::string json_config = ouster::sensor::to_string(_sensor->getMetadata());
    std::ofstream ofs;
    ofs.open(request->metadata_filepath);
    ofs << json_config << std::endl;
    ofs.close();
    if (!ofs) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Failed to save metadata to: %s.",
        request->metadata_filepath.c_str());
    } else {
      RCLCPP_INFO(
        this->get_logger(),
        "Saving metadata to a .json file specifed here: %s",
        request->metadata_filepath.c_str());
    }
  }
}

}  // namespace ros2_ouster
