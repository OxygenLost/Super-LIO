# Super-LIO Offline Bag Reader Design

## Goal

Add an offline execution path for Super-LIO so ROS 2 bag data can be processed without `ros2 bag play` and without ROS timer-driven real-time scheduling.

The design target is the existing Livox ROS 2 bag format already used in this workspace:

- IMU topic: `/livox/imu`
- LiDAR topic: `/livox/lidar`
- LiDAR message type: `livox_ros_driver2/msg/CustomMsg`

The new path must preserve the existing Super-LIO algorithm flow, parameter handling, output topics, and map saving behavior.

## Problem

The current execution path is online-only:

- `ROSWrapper` subscribes to ROS topics
- `super_lio_node` uses a wall timer to call `SuperLIO::process()`
- `rclcpp::spin()` drives message delivery

This means offline data must be replayed through `ros2 bag play`, which introduces avoidable runtime overhead:

- ROS playback scheduling
- DDS transport overhead
- subscription callback dispatch
- wall-timer-driven processing cadence

For long bags, this slows down map generation significantly.

## Chosen Approach

Implement **Option 1**:

- keep `SuperLIO` and `ROSWrapper::sync_measure()` as the core processing path
- add an offline executable that reads ROS 2 bag files directly with `rosbag2_cpp::Reader`
- add explicit message injection APIs to `ROSWrapper`
- reuse the existing online parsing logic by routing offline messages through the same handlers used by subscriptions

This approach minimizes algorithm risk because it does not create a second measurement-building path.

## Non-Goals

This iteration does not attempt to:

- redesign the algorithm state machine
- replace `ROSWrapper` with a pure non-ROS data adapter
- support every LiDAR type in offline mode
- optimize for maximum possible throughput beyond removing online replay overhead
- change the meaning of saved maps or output topics

The first offline mode only needs to support the Livox ROS 2 bags currently used by the user.

## Current Architecture Summary

The relevant current flow is:

1. `ROSWrapper` receives IMU and LiDAR messages through ROS subscriptions.
2. `imuHandler()` and `livoxHandler()` convert messages into internal buffered structures:
   - `std::deque<IMUData> imu_buffer_`
   - `std::deque<LidarData> lidar_buffer_`
3. `SuperLIO::process()` asks `ROSWrapper::sync_measure()` for a synchronized `MeasureGroup`.
4. If a synchronized measurement exists, the normal state machine runs:
   - KF initialization
   - map initialization
   - normal processing
5. Output publishing and map saving remain inside the existing code path.

This means the most stable insertion point for offline mode is **before the buffers**, not after them.

## Design

### 1. Add Offline Injection APIs to `ROSWrapper`

Add new public methods:

- `void feedImu(const sensor_msgs::msg::Imu& msg);`
- `void feedLivox(const livox_ros_driver2::msg::CustomMsg& msg);`

These methods will:

- create a `SharedPtr` copy from the passed message
- forward to the existing private handlers:
  - `imuHandler(...)`
  - `livoxHandler(...)`

This keeps parsing and buffering logic single-sourced.

#### Why this design

Without these methods, offline mode would have to duplicate handler logic or weaken encapsulation by exposing raw buffers. Both are worse:

- exposing buffers leaks internal synchronization assumptions
- duplicating conversion logic risks behavioral drift between online and offline modes

The injection APIs keep `ROSWrapper` responsible for turning ROS messages into buffered internal measurements, regardless of source.

### 2. Add `offline_super_lio_node`

Create a new executable:

- `src/apps/offline_super_lio_node.cpp`

Responsibilities:

- initialize ROS
- construct `ROSWrapper`
- construct `SuperLIO`
- open a ROS 2 bag with `rosbag2_cpp::Reader`
- sequentially read messages from the bag
- dispatch recognized topics into `ROSWrapper`
- actively trigger `SuperLIO::process()`
- save the map on completion

This executable replaces:

- subscription-driven message arrival
- wall-timer-based `process()`
- `rclcpp::spin()`

for the offline path only.

### 3. Offline Read Loop

The reader loop will:

1. open the user-provided bag path
2. iterate over serialized messages in recorded order
3. inspect the topic name
4. deserialize supported topics
5. inject them into `ROSWrapper`

Supported topics in the first implementation:

- `/livox/imu` -> `sensor_msgs::msg::Imu`
- `/livox/lidar` -> `livox_ros_driver2::msg::CustomMsg`

Unsupported topics are ignored.

### 4. Processing Trigger Strategy

Offline mode must explicitly decide when to call `SuperLIO::process()`.

Chosen policy:

- after feeding an IMU message: do not process immediately
- after feeding a LiDAR message: call `SuperLIO::process()`
- after each LiDAR-triggered `process()` call, keep calling `process()` until no further synchronized measure is available

Because `SuperLIO::process()` is currently `void`, the loop cannot directly check a return value.

To support draining without invasive changes, add a lightweight query API on `ROSWrapper`:

- `bool hasSynchronizedMeasure() const;`

This method should answer whether the current buffers contain enough data for `sync_measure()` to succeed:

- at least one LiDAR frame exists
- at least one IMU exists
- newest IMU time covers the front LiDAR frame end time

The offline loop then becomes:

1. feed LiDAR message
2. while `data_wrapper->hasSynchronizedMeasure()`:
   - `lio->process()`

#### Why this matters

If offline mode only calls `process()` once per LiDAR, partially drainable buffered work may be delayed until later messages.

A drain loop more closely matches the intent of “process as fast as data is available”.

### 5. Parameters

The offline executable should preserve the existing parameter model:

- still accept the same YAML file used by `super_lio_node`
- still rely on `ROSWrapper` to load algorithm parameters

Additional offline-only parameters or CLI arguments are needed for:

- bag path

Preferred interface:

- ROS parameter: `offline.bag_path`

For the first implementation, only the bag path is required.

### 6. Output Behavior

Offline mode should preserve the existing outputs:

- `/lio/odom`
- `/lio/path`
- `/lio/cloud_world`
- IMU odometry outputs
- map saving through `saveMap()`

Even though offline mode is not intended for live streaming, preserving outputs keeps behavior consistent and makes debugging easier.

No output topic renaming is required.

### 7. Map Saving

At end of bag:

- call `lio->saveMap()`
- call `lio->printTimeRecord()`
- then `rclcpp::shutdown()`

This matches the current online executable behavior.

No special offline save path is required beyond passing the usual existing map-save parameters.

### 8. Error Handling

The offline executable should fail fast for these conditions:

- bag path missing
- bag cannot be opened
- required topic never appears
- deserialization failure for a required message type

Logging should include:

- topic name
- bag path
- message index or time when practical

Ignored topics should not be warnings.

### 9. Supported Scope

The first implementation supports:

- ROS 2 bags
- Livox IMU + Livox CustomMsg LiDAR input
- current Humble workspace

The design intentionally does not yet support offline parsing for:

- Hesai
- Velodyne
- Ouster
- PointCloud2-based non-Livox inputs

Future non-Livox offline support can be added later by extending the offline dispatcher and reusing existing `stdMsgHandler()` paths.

## File Changes

Expected code changes:

- `src/super_lio/include/ros/ROSWrapper.h`
  - add offline feed methods
  - add synchronized-measure availability query

- `src/super_lio/src/ros/ROSWrapper.cpp`
  - implement feed methods
  - implement synchronized-measure query

- `src/super_lio/src/apps/offline_super_lio_node.cpp`
  - new offline executable

- `src/super_lio/CMakeLists.txt`
  - link `offline_super_lio_node`
  - add `rosbag2_cpp` dependency if needed

- `src/super_lio/package.xml`
  - add runtime/build dependency for `rosbag2_cpp`

## Algorithm Safety

This design intentionally avoids touching:

- `SuperLIO::sync` semantics
- filter math
- undistortion
- map update logic
- output construction

The only new behavior is how ROS messages arrive at the existing buffers.

This sharply reduces regression risk between online and offline modes.

## Alternatives Considered

### A. Direct internal measurement adapter

Create a new offline path that builds `MeasureGroup` directly and bypasses `ROSWrapper`.

Pros:

- potentially cleaner separation
- less ROS coupling
- possibly less message-copy overhead

Cons:

- duplicates logic already embodied in handlers
- higher regression risk
- requires deeper understanding of all supported sensor input conversions

Rejected for the first implementation because it increases scope and risk.

### B. Accelerated `ros2 bag play`

Keep current online path and just replay faster.

Pros:

- no code changes

Cons:

- still pays DDS and playback scheduling overhead
- still coupled to real-time-ish execution model
- does not solve the root cause

Rejected because it does not meet the user’s actual performance goal.

## Verification Plan

Implementation is considered successful only if all of the following are proven:

1. `offline_super_lio_node` builds in the current workspace.
2. It opens the provided Livox ROS 2 bags directly without `ros2 bag play`.
3. It produces normal Super-LIO outputs through the existing processing path.
4. It can save `map.pcd` using the existing save-map parameters.
5. The existing online executable still builds and runs.

Planned verification steps:

- build workspace with the new executable
- run offline node on `0609_1_ros2`
- verify `KF init` and `Map init`
- verify map saved to disk
- run online executable build check to ensure no regression

## Open Decisions

These do not block implementation:

- whether the offline executable should be exposed through `ros2 run` only or also through a launch file
- whether future non-Livox offline support should be added by topic/type autodetection or explicit sensor-mode selection

For this iteration:

- `ros2 run` is sufficient
- Livox-only support is sufficient

## Recommendation

Proceed with this design as the first offline execution implementation.

It is the smallest change that:

- removes online replay overhead
- keeps algorithm risk low
- preserves current behavior
- directly addresses the user’s complaint that online mode is too slow
