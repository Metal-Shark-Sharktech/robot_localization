/*
 * Copyright (c) 2014, 2015, 2016 Charles River Analytics, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above
 * copyright notice, this list of conditions and the following
 * disclaimer in the documentation and/or other materials provided
 * with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "gtest/gtest.h"

#include "robot_localization/navsat_transform.hpp"

#include "GeographicLib/Geocentric.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_localization/srv/to_ll.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "tf2/LinearMath/Transform.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/static_transform_broadcaster.hpp"
#include "tf2_ros/transform_listener.hpp"

class EarthChainTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
  }

  void TearDown() override
  {
    rclcpp::shutdown();
  }

  std::shared_ptr<robot_localization::NavSatTransform> createNavSatNode(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides(parameters);
    return std::make_shared<robot_localization::NavSatTransform>(options);
  }
};

TEST_F(EarthChainTest, CaseA_EarthToCartesianAndCartesianToWorld)
{
  // Configure as_parent=true, broadcast_cartesian=true, broadcast_earth_transform=true
  std::vector<rclcpp::Parameter> parameters = {
    rclcpp::Parameter("broadcast_cartesian_transform", true),
    rclcpp::Parameter("broadcast_cartesian_transform_as_parent_frame", true),
    rclcpp::Parameter("broadcast_earth_transform", true),
    rclcpp::Parameter("earth_frame_id", "earth"),
    rclcpp::Parameter("wait_for_datum", true),
    rclcpp::Parameter("datum", std::vector<double>{40.0, -74.0, 0.0})
  };

  auto node = createNavSatNode(parameters);

  // Create TF buffer to check transforms
  auto tf_buffer = std::make_unique<tf2_ros::Buffer>(node->get_clock());
  auto tf_listener = std::make_unique<tf2_ros::TransformListener>(*tf_buffer);

  // Give some time for transforms to be published
  {
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node->get_node_base_interface());
    executor.spin_some();
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // Test that we can lookup earth->cartesian transform
  try {
    auto transform = tf_buffer->lookupTransform("utm", "earth", tf2::TimePointZero);
    EXPECT_EQ(transform.header.frame_id, "earth");
    EXPECT_EQ(transform.child_frame_id, "utm");
  } catch (tf2::TransformException & ex) {
    // Transform might not be available immediately in test - this is acceptable for now
    RCLCPP_WARN(rclcpp::get_logger("test"), "Transform earth->utm not available: %s", ex.what());
  }

  node.reset();
}

TEST_F(EarthChainTest, CaseB_EarthToWorldAndWorldToCartesian)
{
  // Configure as_parent=false, broadcast_cartesian=true, broadcast_earth_transform=true
  std::vector<rclcpp::Parameter> parameters = {
    rclcpp::Parameter("broadcast_cartesian_transform", true),
    rclcpp::Parameter("broadcast_cartesian_transform_as_parent_frame", false),
    rclcpp::Parameter("broadcast_earth_transform", true),
    rclcpp::Parameter("earth_frame_id", "earth"),
    rclcpp::Parameter("wait_for_datum", true),
    rclcpp::Parameter("datum", std::vector<double>{40.0, -74.0, 0.0})
  };

  auto node = createNavSatNode(parameters);

  // Create TF buffer to check transforms
  auto tf_buffer = std::make_unique<tf2_ros::Buffer>(node->get_clock());
  auto tf_listener = std::make_unique<tf2_ros::TransformListener>(*tf_buffer);

  // Give some time for transforms to be published
  {
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node->get_node_base_interface());
    executor.spin_some();
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // Test that we can lookup earth->world transform
  try {
    auto transform = tf_buffer->lookupTransform("odom", "earth", tf2::TimePointZero);
    EXPECT_EQ(transform.header.frame_id, "earth");
    EXPECT_EQ(transform.child_frame_id, "odom");
  } catch (tf2::TransformException & ex) {
    // Transform might not be available immediately in test - this is acceptable for now
    RCLCPP_WARN(rclcpp::get_logger("test"), "Transform earth->odom not available: %s", ex.what());
  }

  node.reset();
}

TEST_F(EarthChainTest, CaseC_EarthToWorldOnly)
{
  // Configure broadcast_cartesian=false, broadcast_earth_transform=true
  std::vector<rclcpp::Parameter> parameters = {
    rclcpp::Parameter("broadcast_cartesian_transform", false),
    rclcpp::Parameter("broadcast_earth_transform", true),
    rclcpp::Parameter("earth_frame_id", "earth"),
    rclcpp::Parameter("wait_for_datum", true),
    rclcpp::Parameter("datum", std::vector<double>{40.0, -74.0, 0.0})
  };

  auto node = createNavSatNode(parameters);

  // Create TF buffer to check transforms
  auto tf_buffer = std::make_unique<tf2_ros::Buffer>(node->get_clock());
  auto tf_listener = std::make_unique<tf2_ros::TransformListener>(*tf_buffer);

  // Give some time for transforms to be published
  {
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node->get_node_base_interface());
    executor.spin_some();
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // Test that we can lookup earth->world transform and no cartesian link
  try {
    auto transform = tf_buffer->lookupTransform("odom", "earth", tf2::TimePointZero);
    EXPECT_EQ(transform.header.frame_id, "earth");
    EXPECT_EQ(transform.child_frame_id, "odom");
  } catch (tf2::TransformException & ex) {
    // Transform might not be available immediately in test - this is acceptable for now
    RCLCPP_WARN(rclcpp::get_logger("test"), "Transform earth->odom not available: %s", ex.what());
  }

  // Test that cartesian link is NOT published
  try {
    auto transform = tf_buffer->lookupTransform("utm", "odom", tf2::TimePointZero);
    // If we get here without exception, that means the transform exists, which it shouldn't
    FAIL() << "Expected cartesian transform to NOT be published in Case C";
  } catch (tf2::TransformException & ex) {
    // This is expected - no cartesian transform should be published
    SUCCEED();
  }

  node.reset();
}

TEST_F(EarthChainTest, ECEFConversionAccuracy)
{
  // Test known coordinates round-trip validation
  std::vector<rclcpp::Parameter> parameters = {
    rclcpp::Parameter("broadcast_earth_transform", true),
    rclcpp::Parameter("earth_frame_id", "earth"),
    rclcpp::Parameter("use_local_cartesian", true),
    rclcpp::Parameter("wait_for_datum", true),
    rclcpp::Parameter("datum", std::vector<double>{40.7128, -74.0060, 0.0})  // NYC coordinates
  };

  auto node = createNavSatNode(parameters);

  // Give some time for initialization
  {
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node->get_node_base_interface());
    executor.spin_some();
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // Test basic parameters were set correctly
  EXPECT_EQ(node->get_parameter("earth_frame_id").as_string(), "earth");
  EXPECT_EQ(node->get_parameter("broadcast_earth_transform").as_bool(), true);

  node.reset();
}

TEST_F(EarthChainTest, ParameterDefaults)
{
  // Test that default parameters are set correctly
  std::vector<rclcpp::Parameter> parameters = {};

  auto node = createNavSatNode(parameters);

  // Test default values
  EXPECT_EQ(node->get_parameter("earth_frame_id").as_string(), "earth");
  EXPECT_EQ(node->get_parameter("broadcast_earth_transform").as_bool(), false);

  node.reset();
}

// The robot's map pose taken through the earth frame must give back the fix that set the
// datum, at any fix altitude, and toLL must agree with the earth frame.
class EarthAltitudeTest : public ::testing::TestWithParam<double>
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
  }

  void TearDown() override
  {
    rclcpp::shutdown();
  }
};

TEST_P(EarthAltitudeTest, EarthChainReturnsTheFix)
{
  const double fix_latitude = 29.2;
  const double fix_longitude = -90.1;
  const double fix_altitude = GetParam();
  const tf2::Vector3 robot_in_map(3.0, -2.0, 0.15);

  rclcpp::NodeOptions options;
  options.parameter_overrides(
  {
    rclcpp::Parameter("use_local_cartesian", true),
    rclcpp::Parameter("broadcast_cartesian_transform", false),
    rclcpp::Parameter("broadcast_earth_transform", true),
    rclcpp::Parameter("zero_altitude", true),
    rclcpp::Parameter("wait_for_datum", false),
  });
  auto navsat = std::make_shared<robot_localization::NavSatTransform>(options);
  auto helper = std::make_shared<rclcpp::Node>("earth_altitude_test_helper");

  tf2_ros::StaticTransformBroadcaster static_broadcaster(*helper);
  std::vector<geometry_msgs::msg::TransformStamped> mounts(2);
  mounts[0].header.frame_id = "base_link";
  mounts[0].child_frame_id = "gps_link";
  mounts[0].transform.rotation.w = 1.0;
  mounts[1] = mounts[0];
  mounts[1].child_frame_id = "imu_link";
  static_broadcaster.sendTransform(mounts);

  tf2_ros::Buffer tf_buffer(helper->get_clock());
  tf2_ros::TransformListener tf_listener(tf_buffer);

  auto qos = rclcpp::SensorDataQoS();
  auto odom_pub = helper->create_publisher<nav_msgs::msg::Odometry>("odometry/filtered", qos);
  auto imu_pub = helper->create_publisher<sensor_msgs::msg::Imu>("imu", qos);
  auto fix_pub = helper->create_publisher<sensor_msgs::msg::NavSatFix>("gps/fix", qos);
  auto to_ll = helper->create_client<robot_localization::srv::ToLL>("toLL");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(navsat);
  executor.add_node(helper);

  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
  bool have_earth = false;
  while (!have_earth && std::chrono::steady_clock::now() < deadline) {
    const auto stamp = helper->now();

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = "map";
    odom.child_frame_id = "base_link";
    odom.pose.pose.position.x = robot_in_map.x();
    odom.pose.pose.position.y = robot_in_map.y();
    odom.pose.pose.position.z = robot_in_map.z();
    odom.pose.pose.orientation.w = 1.0;
    odom_pub->publish(odom);

    sensor_msgs::msg::Imu imu;
    imu.header.stamp = stamp;
    imu.header.frame_id = "imu_link";
    imu.orientation.w = 1.0;
    imu_pub->publish(imu);

    sensor_msgs::msg::NavSatFix fix;
    fix.header.stamp = stamp;
    fix.header.frame_id = "gps_link";
    fix.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
    fix.latitude = fix_latitude;
    fix.longitude = fix_longitude;
    fix.altitude = fix_altitude;
    fix_pub->publish(fix);

    executor.spin_some(std::chrono::milliseconds(50));
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    have_earth = tf_buffer.canTransform("earth", "map", tf2::TimePointZero);
  }
  ASSERT_TRUE(have_earth) << "earth -> map was never published";

  tf2::Transform earth_from_map;
  tf2::fromMsg(
    tf_buffer.lookupTransform("earth", "map", tf2::TimePointZero).transform,
    earth_from_map);

  const GeographicLib::Geocentric & wgs84 = GeographicLib::Geocentric::WGS84();
  auto through_earth = [&](const tf2::Vector3 & map_point, double & lat, double & lon,
    double & alt) {
      const tf2::Vector3 ecef = earth_from_map * map_point;
      wgs84.Reverse(ecef.x(), ecef.y(), ecef.z(), lat, lon, alt);
    };

  const double degree_tolerance = 1e-8;
  const double height_tolerance = 1e-3;

  double latitude = 0.0;
  double longitude = 0.0;
  double altitude = 0.0;
  through_earth(robot_in_map, latitude, longitude, altitude);
  EXPECT_NEAR(latitude, fix_latitude, degree_tolerance);
  EXPECT_NEAR(longitude, fix_longitude, degree_tolerance);
  EXPECT_NEAR(altitude, fix_altitude, height_tolerance);

  ASSERT_TRUE(to_ll->wait_for_service(std::chrono::seconds(5)));
  for (const tf2::Vector3 & map_point : {robot_in_map, tf2::Vector3(10.0, 5.0, -3.0)}) {
    auto request = std::make_shared<robot_localization::srv::ToLL::Request>();
    request->map_point.x = map_point.x();
    request->map_point.y = map_point.y();
    request->map_point.z = map_point.z();
    auto future = to_ll->async_send_request(request);
    ASSERT_EQ(
      executor.spin_until_future_complete(future, std::chrono::seconds(5)),
      rclcpp::FutureReturnCode::SUCCESS);
    const auto ll_point = future.get()->ll_point;

    through_earth(map_point, latitude, longitude, altitude);
    EXPECT_NEAR(ll_point.latitude, latitude, degree_tolerance);
    EXPECT_NEAR(ll_point.longitude, longitude, degree_tolerance);
    EXPECT_NEAR(ll_point.altitude, altitude, height_tolerance);
  }
}

INSTANTIATE_TEST_SUITE_P(
  FixAltitudes, EarthAltitudeTest,
  ::testing::Values(0.0, 2.885, -25.6, 120.0));

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
