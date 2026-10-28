# Gazebo snapshot timing regression

Use both parameters for Gazebo `gpu_lidar` snapshots:

```yaml
lio.sensor.lidar_type: 8
lio.sensor.is_gazebo: true
```

`lio.sensor.is_gazebo` defaults to `false`. Real LiDAR should use `false` and its
actual `lidar_type` (for example, `1` for Livox CustomMsg). This is a startup
parameter; restart the node after changing the configuration. The PX4 bridge's
Gazebo mapping and relocation configurations enable it; Super-LIO's real Livox
configuration explicitly disables it. Extrinsics and filtering settings are unchanged.

With `true` and format `8`, every point uses the cloud header time. IMU prediction
reaches that time through interpolation, and points receive only the fixed
LiDAR-to-IMU transform. Initialization uses the snapshot time too. Other message
formats retain their real point timestamps even if this switch is enabled.

With `false`, the original scan synchronization, per-point motion compensation,
and map/relocation initialization paths remain active. Format `8` also retains
its original indexed scan timing when disabled. The real-scan `tau` translation
fix and duplicate-timestamp/iterator protections remain shared bug fixes. Real
mode does not synthesize an IMU sample at the cloud endpoint.

From the ROS workspace root, build and run the regression tests:

```bash
colcon build --packages-select super_lio px4_bridge --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
colcon test --packages-select super_lio --event-handlers console_direct+
colcon test-result --verbose
```

The eight GTests exercise the production conversion, synchronization, prediction,
undistortion and odometry publication code. They cover:

- A snapshot undergoing translation and yaw rates of 0, 6 and -12 rad/s: IMU
  position/orientation advance, while the cloud remains the same rigid shape.
- Real scan translation at an interval interior and exact endpoints, out-of-range
  point times, duplicate/older IMUs, and a single-state history.
- IMU prediction during map initialization.
- Interpolated IMU acceleration/angular velocity at the cloud time, multiple
  clouds within one IMU interval, retention of the later IMU, and the published
  `/lio/odom` timestamp.
- Clouds preceding available IMU history and duplicate cloud timestamps.
- Default-off indexed timing and original IMU batching, real Livox hardware point
  timestamps with an explicit `false`, and original real-scan map initialization.

For the Gazebo acceptance run, keep the existing extrinsics and execute
**stationary → rotate in place → translate while rotating**. Record the raw
cloud, IMU, `/lio/cloud_world`, and `/lio/odom` with their header timestamps.
For geometric comparison, publish the dense processed cloud with
`lio.output.dense: true` and `lio.output.pub_point_step: 1`; compare only raw
points retained by the existing range filter and `lio.sensor.filter_rate`.
Match frames by sampling timestamp, not arrival order. Account for float
rounding of published timestamps (at most approximately 1 ns here).

Fit a best rigid transform from the retained raw points to the processed points
and report RMS/max residual versus yaw rate. Snapshot processing should add only
floating-point error, not the earlier 20–31 cm deformation. Also compare the
thickness and layering of the same wall across yaw rates. RViz single-frame
viewing helps inspect individual clouds; accumulated walls are needed to judge
inter-frame alignment. The synthetic tests do not replace this scene-level run.
