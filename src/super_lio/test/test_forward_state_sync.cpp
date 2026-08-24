#include <gtest/gtest.h>

#include "lio/ESKF.h"

namespace {

using BASIC::V3;
using LI2Sup::DynamicState;
using LI2Sup::ESKF;
using LI2Sup::IMUData;
using LI2Sup::SysState;

TEST(ForwardStateSync, UsesExactScanEndImuForNextTrapezoid)
{
  ESKF filter;
  ESKF::Options options;
  filter.SetInitialConditions(
      options, V3::Zero(), V3::Zero(), 1.0F, V3::Zero());

  SysState scan_state(2.0);
  scan_state.p = V3(1.0, 2.0, 3.0);
  scan_state.v = V3(0.5, 0.0, 0.0);
  filter.SetX(scan_state);
  filter.init_ = true;

  // Seed a deliberately stale forward measurement, as happened when IMU
  // callbacks ran before the scan timer.
  DynamicState ignored_imu_state, ignored_robot_state;
  IMUData stale;
  stale.secs = 1.5;
  stale.acc = V3(100.0, 0.0, 0.0);
  EXPECT_FALSE(filter.Predict(stale, ignored_imu_state, ignored_robot_state));

  IMUData scan_end;
  scan_end.secs = 2.0;
  scan_end.acc = V3(2.0, 0.0, 0.0);
  filter.SynchronizeForwardState(scan_end);

  IMUData next;
  next.secs = 2.1;
  next.acc = V3(4.0, 0.0, 0.0);
  DynamicState imu_state, robot_state;
  ASSERT_TRUE(filter.Predict(next, imu_state, robot_state));

  // Trapezoidal acceleration is (2 + 4) / 2 = 3 m/s^2. The stale 100
  // m/s^2 measurement must not contribute after synchronization.
  EXPECT_NEAR(imu_state.time, 2.1, 1e-12);
  EXPECT_NEAR(imu_state.p.x(), 1.0 + 0.5 * 0.1 + 0.5 * 3.0 * 0.01, 1e-6);
  EXPECT_NEAR(imu_state.p.y(), 2.0, 1e-6);
  EXPECT_NEAR(imu_state.p.z(), 3.0, 1e-6);
  EXPECT_NEAR(imu_state.v.x(), 0.5 + 3.0 * 0.1, 1e-6);
}

}  // namespace
