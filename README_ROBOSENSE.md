# DLIO with Robosense LiDAR Support

This fork adds support for Robosense MEMS LiDARs to the Direct LiDAR-Inertial Odometry (DLIO) system.

## Supported Robosense LiDAR Models

The following Robosense MEMS LiDARs are supported:

- **RS-LiDAR-AC1** - 5-line MEMS LiDAR (latest model)
- **RS-LiDAR-E1R** - 5-line MEMS LiDAR (previous model)
- **RS-LiDAR-M1** - 5-line MEMS LiDAR
- **RS-LiDAR-Airy** - 96-line MEMS LiDAR

These LiDARs use the same point cloud format and are automatically detected by the system.

## Key Features

### Automatic Sensor Detection

DLIO automatically detects Robosense LiDARs by checking for both `timestamp` (double) and `ring` (uint16_t) fields in the point cloud data. No manual configuration of sensor type is needed.

### Point Cloud Format

Robosense LiDARs use the following point cloud format:

```cpp
struct Point {
  float x, y, z;           // 3D coordinates
  float intensity;         // Reflectivity
  uint16_t ring;          // Laser ring number
  double timestamp;        // Absolute timestamp in seconds
}
```

**Key Characteristics:**
- `timestamp` field is `double` type (unlike Velodyne's `float time` or Ouster's `uint32_t t`)
- Timestamp represents **absolute time** in seconds
- The system automatically converts absolute timestamps to relative time for motion correction

### Deskewing (Motion Correction)

DLIO implements proper deskewing for Robosense LiDARs:
- Points are sorted by timestamp
- IMU data is integrated to estimate the sensor trajectory during the scan
- Each point is transformed to account for sensor motion
- This provides accurate odometry even during fast motion

## Usage

### Building

```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select direct_lidar_inertial_odometry
source install/setup.bash
```

### Running with AC1/E1R/M1 (5-line)

```bash
ros2 launch direct_lidar_inertial_odometry dlio_robosense_ac1.launch.py rviz:=true
```

### Running with Airy (96-line)

```bash
ros2 launch direct_lidar_inertial_odometry dlio_robosense_airy.launch.py rviz:=true
```

### Custom Topics

You can specify custom topic names:

```bash
ros2 launch direct_lidar_inertial_odometry dlio_robosense_ac1.launch.py \
    pointcloud_topic:=/your_lidar_topic \
    imu_topic:=/your_imu_topic \
    rviz:=true
```

## Configuration

### AC1/E1R/M1 Configuration

The AC1 configuration is optimized for 5-line MEMS LiDAR:
- Smaller voxel filter resolution (0.2m) for better feature preservation
- More aggressive keyframe thresholds (0.5m distance, 30° rotation)
- Lower minimum point requirement (32 points) for GICP

Configuration files:
- `cfg/dlio_robosense_ac1.yaml` - Sensor calibration and extrinsics
- `cfg/params_robosense_ac1.yaml` - DLIO algorithm parameters

### Airy Configuration

The Airy configuration uses standard parameters suitable for 96-line LiDAR:
- Standard voxel filter resolution (0.25m)
- Standard keyframe thresholds (1.0m distance, 45° rotation)
- Standard GICP parameters (64 minimum points)

Configuration files:
- `cfg/dlio_robosense_airy.yaml` - Sensor calibration and extrinsics
- `cfg/params_robosense_airy.yaml` - DLIO algorithm parameters

### Extrinsics Calibration

Edit the configuration files to set the extrinsics between LiDAR, IMU, and robot baselink:

```yaml
extrinsics/baselink2imu/t: [ 0.0, 0.0, 0.0 ]     # Translation (x, y, z)
extrinsics/baselink2imu/R: [ 1., 0., 0.,          # Rotation matrix
                             0., 1., 0.,
                             0., 0., 1. ]

extrinsics/baselink2lidar/t: [ 0.0, 0.0, 0.0 ]
extrinsics/baselink2lidar/R: [ 1., 0., 0.,
                               0., 1., 0.,
                               0., 0., 1. ]
```

## Robosense ROS Driver Setup

You need to install and configure the Robosense ROS driver:

### Installation

```bash
cd ~/ros2_ws/src
git clone https://github.com/RoboSense/rslidar_sdk.git
cd ~/ros2_ws
colcon build --symlink-install --packages-select rslidar_sdk
```

### Driver Configuration

Ensure the Robosense driver outputs point clouds with the correct format:

```yaml
msg_source: 1                           # 1: LiDAR, 5: Packet
send_point_cloud_ros: true
ros_send_point_cloud_topic: /rs_lidar/points
timestamp_type: 0                       # 0: Absolute time in seconds
```

## Implementation Details

### Changes to DLIO

1. **Sensor Type Enum** (`include/dlio/dlio.h:54`)
   - Added `ROBOSENSE` to the `SensorType` enum

2. **Sensor Auto-Detection** (`src/dlio/odom.cc:508-537`)
   - Detects Robosense by checking for both `timestamp` and `ring` fields
   - Distinguishes from HESAI (timestamp only) and LIVOX (timestamp with large values)

3. **Deskewing Logic** (`src/dlio/odom.cc:657-665`)
   - Added Robosense-specific timestamp extraction
   - Handles absolute timestamps in seconds
   - Similar to HESAI but optimized for Robosense's timestamp format

4. **Configuration Files**
   - `cfg/dlio_robosense_ac1.yaml` - AC1 LiDAR settings
   - `cfg/params_robosense_ac1.yaml` - AC1 algorithm parameters
   - `cfg/dlio_robosense_airy.yaml` - Airy LiDAR settings
   - `cfg/params_robosense_airy.yaml` - Airy algorithm parameters

5. **Launch Files**
   - `launch/dlio_robosense_ac1.launch.py` - AC1 launcher
   - `launch/dlio_robosense_airy.launch.py` - Airy launcher

### Why Robosense Requires Special Implementation

1. **Timestamp Format Difference**
   - Velodyne: `float time` (relative time since scan start)
   - Ouster: `uint32_t t` (nanoseconds since scan start)
   - HESAI: `double timestamp` (absolute time in seconds)
   - Robosense: `double timestamp` (absolute time in seconds) + `ring` field

2. **Point Cloud Structure**
   - Robosense includes both `timestamp` and `ring` fields
   - This combination is unique and allows reliable auto-detection

3. **MEMS LiDAR Characteristics**
   - Non-rotating scan pattern (unlike traditional spinning LiDARs)
   - High scan rate (10Hz standard)
   - Accurate per-point timestamps are critical for motion correction

## Troubleshooting

### Sensor Not Detected

Check that the point cloud has the correct fields:
```bash
ros2 topic echo /rs_lidar/points --field fields
```

You should see both `timestamp` (FLOAT64) and `ring` (UINT16) fields.

### Bad Time Synchronization Error

This error indicates that the LiDAR and IMU timestamps are not synchronized:
- Ensure both sensors are using the same time source
- Check that the Robosense driver is publishing absolute timestamps
- Verify IMU topic is publishing data

### Poor Odometry Quality

For AC1/E1R/M1 (5-line LiDAR):
- Ensure the environment has sufficient geometric features
- Adjust `odom/preprocessing/voxelFilter/res` if needed
- Try reducing `odom/keyframe/threshD` for more frequent keyframes

For Airy (96-line LiDAR):
- Standard parameters should work well in most environments
- Adjust voxel filter resolution based on environment scale

## Credits

- Original DLIO by VECTR Lab, UCLA
- Robosense support implementation based on robosense_fast_lio
- Robosense LiDAR technology by RoboSense

## References

- [Original DLIO Repository](https://github.com/vectr-ucla/direct_lidar_inertial_odometry)
- [Robosense Official Site](https://www.robosense.ai/)
- [Robosense ROS SDK](https://github.com/RoboSense/rslidar_sdk)
