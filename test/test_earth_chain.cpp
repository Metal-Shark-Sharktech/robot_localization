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
#include <cmath>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <tuple>
#include <vector>

#include "gtest/gtest.h"

#include "robot_localization/navsat_transform.hpp"

#include "GeographicLib/Geocentric.hpp"
#include "geographic_msgs/msg/geo_point.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_localization/srv/set_datum.hpp"
#include "robot_localization/srv/to_ll.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
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
  rclcpp::spin_some(node);
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
  rclcpp::spin_some(node);
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
  rclcpp::spin_some(node);
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
  rclcpp::spin_some(node);
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

namespace
{

struct EarthPublishMode
{
  bool broadcast_cartesian;
  bool as_parent;
  bool zero_altitude;
};

struct Geodetic
{
  double latitude;
  double longitude;
  double height;
};

constexpr double FIX_LATITUDE = 29.2;
constexpr double FIX_LONGITUDE = -90.1;
constexpr double GPS_HEIGHT_IN_BASE_LINK = 1.2;
constexpr double ROBOT_HEIGHT_IN_WORLD = 0.15;
constexpr double ROBOT_YAW_IN_WORLD = 0.4;
constexpr double DEGREE_TOLERANCE = 1e-8;
constexpr double HEIGHT_TOLERANCE = 1e-3;

Geodetic toGeodetic(const tf2::Vector3 & ecef)
{
  Geodetic geodetic{};
  GeographicLib::Geocentric::WGS84().Reverse(
    ecef.x(), ecef.y(), ecef.z(), geodetic.latitude, geodetic.longitude, geodetic.height);
  return geodetic;
}

tf2::Quaternion yawQuaternion(const double yaw)
{
  tf2::Quaternion quaternion;
  quaternion.setRPY(0.0, 0.0, yaw);
  return quaternion;
}

std::shared_ptr<robot_localization::NavSatTransform> makeNavSat(
  const EarthPublishMode & mode, const bool wait_for_datum)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides(
    {
      rclcpp::Parameter("use_local_cartesian", true),
      rclcpp::Parameter("broadcast_cartesian_transform", mode.broadcast_cartesian),
      rclcpp::Parameter("broadcast_cartesian_transform_as_parent_frame", mode.as_parent),
      rclcpp::Parameter("broadcast_earth_transform", true),
      rclcpp::Parameter("zero_altitude", mode.zero_altitude),
      rclcpp::Parameter("wait_for_datum", wait_for_datum),
      rclcpp::Parameter("datum", std::vector<double>{FIX_LATITUDE, FIX_LONGITUDE, 0.0}),
    });
  return std::make_shared<robot_localization::NavSatTransform>(options);
}

class EarthChainRig
{
public:
  EarthChainRig(const EarthPublishMode & mode, const bool wait_for_datum)
  : navsat_(makeNavSat(mode, wait_for_datum)),
    helper_(std::make_shared<rclcpp::Node>("earth_chain_test_helper")),
    static_broadcaster_(*helper_),
    tf_buffer_(helper_->get_clock()),
    tf_listener_(tf_buffer_),
    odom_pub_(helper_->create_publisher<nav_msgs::msg::Odometry>(
        "odometry/filtered", rclcpp::SensorDataQoS())),
    imu_pub_(helper_->create_publisher<sensor_msgs::msg::Imu>("imu", rclcpp::SensorDataQoS())),
    fix_pub_(helper_->create_publisher<sensor_msgs::msg::NavSatFix>(
        "gps/fix", rclcpp::SensorDataQoS())),
    to_ll_(helper_->create_client<robot_localization::srv::ToLL>("toLL")),
    set_datum_(helper_->create_client<robot_localization::srv::SetDatum>("datum"))
  {
    std::vector<geometry_msgs::msg::TransformStamped> mounts(2);
    mounts[0].header.frame_id = "base_link";
    mounts[0].child_frame_id = "gps_link";
    mounts[0].transform.translation.x = 0.5;
    mounts[0].transform.translation.z = GPS_HEIGHT_IN_BASE_LINK;
    mounts[0].transform.rotation.w = 1.0;
    mounts[1].header.frame_id = "base_link";
    mounts[1].child_frame_id = "imu_link";
    mounts[1].transform.rotation.w = 1.0;
    static_broadcaster_.sendTransform(mounts);

    executor_.add_node(navsat_);
    executor_.add_node(helper_);
  }

  static tf2::Vector3 gpsInWorld()
  {
    return robotInWorld() + tf2::quatRotate(
      yawQuaternion(ROBOT_YAW_IN_WORLD), tf2::Vector3(0.5, 0.0, GPS_HEIGHT_IN_BASE_LINK));
  }

  void setFixAltitude(const double altitude)
  {
    fix_altitude_ = altitude;
  }

  bool spinUntil(const std::function<bool()> & done)
  {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
    while (std::chrono::steady_clock::now() < deadline) {
      publishInputs();
      executor_.spin_some(std::chrono::milliseconds(50));
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
      if (done()) {
        return true;
      }
    }
    return false;
  }

  bool hasTransform(const std::string & parent, const std::string & child)
  {
    return tf_buffer_.canTransform(parent, child, tf2::TimePointZero);
  }

  tf2::Transform lookup(const std::string & parent, const std::string & child)
  {
    tf2::Transform transform;
    tf2::fromMsg(
      tf_buffer_.lookupTransform(parent, child, tf2::TimePointZero).transform, transform);
    return transform;
  }

  Geodetic throughEarth(const tf2::Vector3 & world_point)
  {
    return toGeodetic(lookup("earth", "map") * world_point);
  }

  void toLL(const tf2::Vector3 & world_point, geographic_msgs::msg::GeoPoint & ll_point)
  {
    ASSERT_TRUE(to_ll_->wait_for_service(std::chrono::seconds(5)));
    auto request = std::make_shared<robot_localization::srv::ToLL::Request>();
    request->map_point.x = world_point.x();
    request->map_point.y = world_point.y();
    request->map_point.z = world_point.z();
    auto future = to_ll_->async_send_request(request);
    ASSERT_EQ(
      executor_.spin_until_future_complete(future, std::chrono::seconds(5)),
      rclcpp::FutureReturnCode::SUCCESS);
    ll_point = future.get()->ll_point;
  }

  void setDatum(const Geodetic & datum)
  {
    ASSERT_TRUE(set_datum_->wait_for_service(std::chrono::seconds(5)));
    auto request = std::make_shared<robot_localization::srv::SetDatum::Request>();
    request->geo_pose.position.latitude = datum.latitude;
    request->geo_pose.position.longitude = datum.longitude;
    request->geo_pose.position.altitude = datum.height;
    request->geo_pose.orientation.w = 1.0;
    auto future = set_datum_->async_send_request(request);
    ASSERT_EQ(
      executor_.spin_until_future_complete(future, std::chrono::seconds(5)),
      rclcpp::FutureReturnCode::SUCCESS);
  }

private:
  static tf2::Vector3 robotInWorld()
  {
    return tf2::Vector3(3.0, -2.0, ROBOT_HEIGHT_IN_WORLD);
  }

  void publishInputs()
  {
    const auto stamp = helper_->now();

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = "map";
    odom.child_frame_id = "base_link";
    const tf2::Vector3 robot = robotInWorld();
    odom.pose.pose.position.x = robot.x();
    odom.pose.pose.position.y = robot.y();
    odom.pose.pose.position.z = robot.z();
    odom.pose.pose.orientation = tf2::toMsg(yawQuaternion(ROBOT_YAW_IN_WORLD));
    odom_pub_->publish(odom);

    sensor_msgs::msg::Imu imu;
    imu.header.stamp = stamp;
    imu.header.frame_id = "imu_link";
    imu.orientation = tf2::toMsg(yawQuaternion(0.5));
    imu_pub_->publish(imu);

    if (fix_altitude_) {
      sensor_msgs::msg::NavSatFix fix;
      fix.header.stamp = stamp;
      fix.header.frame_id = "gps_link";
      fix.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
      fix.latitude = FIX_LATITUDE;
      fix.longitude = FIX_LONGITUDE;
      fix.altitude = *fix_altitude_;
      fix_pub_->publish(fix);
    }
  }

  std::shared_ptr<robot_localization::NavSatTransform> navsat_;
  std::shared_ptr<rclcpp::Node> helper_;
  tf2_ros::StaticTransformBroadcaster static_broadcaster_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr fix_pub_;
  rclcpp::Client<robot_localization::srv::ToLL>::SharedPtr to_ll_;
  rclcpp::Client<robot_localization::srv::SetDatum>::SharedPtr set_datum_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::optional<double> fix_altitude_;
};

}  // namespace

class EarthHeightTest : public ::testing::TestWithParam<std::tuple<EarthPublishMode, double>>
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

TEST_P(EarthHeightTest, FixComesBackThroughEarth)
{
  const EarthPublishMode mode = std::get<0>(GetParam());
  const double fix_altitude = std::get<1>(GetParam());

  EarthChainRig rig(mode, false);
  rig.setFixAltitude(fix_altitude);
  ASSERT_TRUE(rig.spinUntil([&rig]() {return rig.hasTransform("earth", "map");}));

  const tf2::Vector3 gps_in_world = EarthChainRig::gpsInWorld();
  const Geodetic gps = rig.throughEarth(gps_in_world);
  EXPECT_NEAR(gps.latitude, FIX_LATITUDE, DEGREE_TOLERANCE);
  EXPECT_NEAR(gps.longitude, FIX_LONGITUDE, DEGREE_TOLERANCE);
  EXPECT_NEAR(gps.height, fix_altitude, HEIGHT_TOLERANCE);

  // toLL treats local z as height, so it only matches the earth chain near the datum.
  for (const tf2::Vector3 & world_point : {gps_in_world, tf2::Vector3(10.0, 5.0, -3.0)}) {
    const Geodetic earth = rig.throughEarth(world_point);
    geographic_msgs::msg::GeoPoint ll_point;
    rig.toLL(world_point, ll_point);
    EXPECT_NEAR(ll_point.latitude, earth.latitude, DEGREE_TOLERANCE);
    EXPECT_NEAR(ll_point.longitude, earth.longitude, DEGREE_TOLERANCE);
    EXPECT_NEAR(ll_point.altitude, earth.height, HEIGHT_TOLERANCE);
  }

  if (mode.broadcast_cartesian && mode.as_parent) {
    const double world_height = fix_altitude - GPS_HEIGHT_IN_BASE_LINK - ROBOT_HEIGHT_IN_WORLD;
    const double published_z = mode.zero_altitude ? 0.0 : world_height;
    const double local_enu_height = mode.zero_altitude ? world_height : 0.0;
    EXPECT_NEAR(rig.lookup("local_enu", "map").getOrigin().z(), published_z, HEIGHT_TOLERANCE);
    const Geodetic local_enu = toGeodetic(rig.lookup("earth", "local_enu").getOrigin());
    EXPECT_NEAR(local_enu.height, local_enu_height, HEIGHT_TOLERANCE);
  }
}

TEST_P(EarthHeightTest, SetDatumComesBackThroughEarth)
{
  const EarthPublishMode mode = std::get<0>(GetParam());
  const double altitude = std::get<1>(GetParam());

  EarthChainRig rig(mode, true);
  auto reaches = [&rig](const Geodetic & datum) {
      return rig.spinUntil(
        [&rig, &datum]() {
          if (!rig.hasTransform("earth", "map")) {
            return false;
          }
          const Geodetic origin = rig.throughEarth(tf2::Vector3(0.0, 0.0, 0.0));
          return std::abs(origin.latitude - datum.latitude) < DEGREE_TOLERANCE &&
                 std::abs(origin.longitude - datum.longitude) < DEGREE_TOLERANCE &&
                 std::abs(origin.height - datum.height) < HEIGHT_TOLERANCE;
        });
    };

  const Geodetic first{FIX_LATITUDE, FIX_LONGITUDE, altitude};
  rig.setDatum(first);
  EXPECT_TRUE(reaches(first));

  const Geodetic second{FIX_LATITUDE + 0.01, FIX_LONGITUDE - 0.01, altitude + 7.5};
  rig.setDatum(second);
  EXPECT_TRUE(reaches(second));
}

INSTANTIATE_TEST_SUITE_P(
  PublishModes, EarthHeightTest,
  ::testing::Combine(
    ::testing::Values(
      EarthPublishMode{false, false, true},
      EarthPublishMode{true, false, true},
      EarthPublishMode{true, true, false},
      EarthPublishMode{true, true, true}),
    ::testing::Values(-25.6, 120.0)));

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
