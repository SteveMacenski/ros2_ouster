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

#ifndef ROS2_OUSTER__PROCESSORS__POINTCLOUD_PROCESSOR_HPP_
#define ROS2_OUSTER__PROCESSORS__POINTCLOUD_PROCESSOR_HPP_

#include <vector>
#include <memory>
#include <string>
#include <utility>

#include "rclcpp/qos.hpp"

#include "ros2_ouster/conversions.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"

#include "ros2_ouster/client/client.h"
#include "ros2_ouster/client/lidar_scan.h"
#include "ros2_ouster/client/point.h"
#include "ros2_ouster/interfaces/data_processor_interface.hpp"
#include "ros2_ouster/full_rotation_accumulator.hpp"

using Cloud = pcl::PointCloud<ouster_ros::Point>;

namespace sensor
{
/**
 * @class sensor::PointcloudProcessor
 * @brief A data processor interface implementation of a processor
 * for creating Pointclouds in the
 * driver in ROS2.
 */
class PointcloudProcessor : public ros2_ouster::DataProcessorInterface
{
public:
  /**
   * @brief A constructor for sensor::PointcloudProcessor
   * @param node Node for creating interfaces
   * @param mdata metadata about the sensor
   * @param frame frame_id to use for messages
   */
  PointcloudProcessor(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr node,
    const ouster::sensor::sensor_info & mdata,
    const std::string & frame,
    const rclcpp::QoS & qos,
    const ouster::sensor::packet_format & pf,
    std::shared_ptr<sensor::FullRotationAccumulator> fullRotationAccumulator)
  : DataProcessorInterface(), _node(node), _frame(frame)
  {
    _node->declare_parameter("pointcloud_filter_zero_points", false);
    _node->get_parameter("pointcloud_filter_zero_points", _filter_zero_points);

    _fullRotationAccumulator = fullRotationAccumulator;
    _height = mdata.format.pixels_per_column;
    _width = mdata.format.columns_per_frame;
    _xyz_lut = ouster::make_xyz_lut(mdata);
    _cloud = std::make_unique<Cloud>(_width, _height);
    _cloud_filtered = _filter_zero_points
      ? std::make_unique<Cloud>(_width * _height, 1)
      : nullptr;
    _pub = _node->create_publisher<sensor_msgs::msg::PointCloud2>(
      "points", qos);
  }

  /**
   * @brief A destructor clearing memory allocated
   */
  ~PointcloudProcessor()
  {
    _pub.reset();
  }

  /**
   * @brief Process method to create pointcloud
   * @param data the packet data
   */
  bool process(const uint8_t * data, const uint64_t override_ts) override
  {
    if (!_fullRotationAccumulator->isBatchReady()) {
      return true;
    }

    ros2_ouster::toCloud(
      _xyz_lut, _fullRotationAccumulator->getTimestamp(),
      *_fullRotationAccumulator->getLidarScan(), *_cloud);

    if (_filter_zero_points) {
      _cloud_filtered->points.clear();

      for (const auto & p : _cloud->points) {
        if (p.x == 0.0 && p.y == 0.0 && p.z == 0.0) {
          continue;
        }
        _cloud_filtered->points.push_back(p);
      }

      _cloud_filtered->width = _cloud_filtered->points.size();
      _cloud_filtered->height = 1;
    }

    _pub->publish(
      ros2_ouster::toMsg(
        *(_filter_zero_points ? _cloud_filtered : _cloud),
        _fullRotationAccumulator->getTimestamp(),
        _frame, override_ts));

    return true;
  }

  /**
   * @brief Activating processor from lifecycle state transitions
   */
  void onActivate() override
  {
    _pub->on_activate();
  }

  /**
   * @brief Deactivating processor from lifecycle state transitions
   */
  void onDeactivate() override
  {
    _pub->on_deactivate();
  }

private:
  std::unique_ptr<Cloud> _cloud;
  std::unique_ptr<Cloud> _cloud_filtered;
  bool _filter_zero_points;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr _pub;
  rclcpp_lifecycle::LifecycleNode::SharedPtr _node;
  ouster::XYZLut _xyz_lut;
  std::string _frame;
  uint32_t _height;
  uint32_t _width;
  std::shared_ptr<sensor::FullRotationAccumulator> _fullRotationAccumulator;
};

}  // namespace sensor

#endif  // ROS2_OUSTER__PROCESSORS__POINTCLOUD_PROCESSOR_HPP_
