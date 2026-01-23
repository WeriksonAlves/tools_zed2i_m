# ZED2i MATLAB ROS 2 Wrapper

MATLAB wrapper for the **ZED2i stereo camera** using **ROS 2**, following the
robotics laboratory class architecture pattern (`@Class` folders).

This repository provides a **stable, modular, and extensible MATLAB interface**
to access **RGB images, depth maps, camera calibration, IMU, pose/odometry,
and point clouds** from a ZED2i camera via ROS 2.

The implementation is designed for **educational and experimental use in
robotics laboratories**, emphasizing clarity, robustness, and reusability.

---

## Overview

This project implements a MATLAB interface to the **ZED2i** camera through the
official **ZED ROS 2 wrapper**, targeting **laboratory classes, research
prototyping, and perception experiments**.

The design follows the same architectural conventions used by existing
laboratory sensor and robot classes, enabling:

* consistent APIs across sensors,
* predictable lifecycle management,
* safe integration into control, perception, and mapping pipelines,
* optional and modular activation of sensors (IMU, pose, point cloud).

The implementation has been tested with **MATLAB R2025a** and ROS 2.

---

## Features

### Core

* RGB image acquisition
* Depth image acquisition (metric, meters)
* Camera calibration retrieval via ROS 2 `CameraInfo`
* Lazy caching of calibration parameters
* Aggregated, lab-style access via `getSensorData`
* Optional conversion to MATLAB `cameraIntrinsics`
* Efficient real-time RGB-D visualization

### Extended Sensors (Optional)

* IMU access (`sensor_msgs/Imu`)

  * Angular velocity
  * Linear acceleration
* Pose / odometry access (`nav_msgs/Odometry`)

  * Position (XYZ)
  * Orientation (Euler angles in radians)
* Registered point cloud access (`sensor_msgs/PointCloud2`)
* Real-time point cloud visualization with fixed spatial scale

### Design

* Minimal, modular, and extensible class design
* Explicit enable/disable of optional sensors via constructor
* Stable output structs for safe downstream use
* Robust error handling and defensive defaults

---

## Requirements

### MATLAB

* MATLAB **R2025a**
* **Robotics System Toolbox** (ROS 2 support required)
* *(Optional)* **Computer Vision Toolbox**

  * Required only for:

    * `cameraIntrinsics`
    * `pointCloud / pcshow` visualization

### ROS 2 and ZED

* ROS 2 environment properly sourced
* ZED ROS 2 wrapper installed and running
* ZED SDK correctly installed
* A ZED2i camera connected and detected

> ⚠️ This MATLAB wrapper **does not start the ZED node**.
> The ZED ROS 2 wrapper must already be running and publishing topics.

---

## ZED ROS 2 Wrapper (Expected Setup)

Before using this MATLAB wrapper, ensure that the ZED ROS 2 node is active.

Typical launch example (outside MATLAB):

```bash
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2i
```

Expected topics include:

```bash
/zed/zed_node/left/image_rect_color
/zed/zed_node/depth/depth_registered
/zed/zed_node/left/camera_info
/zed/zed_node/imu/data
/zed/zed_node/odom
/zed/zed_node/point_cloud/cloud_registered
```

These topics are **consumed directly** by the MATLAB class.

---

## ROS 2 Topics Used

Default topics (configurable via constructor):

| Data        | Topic                                        |
| ----------- | -------------------------------------------- |
| RGB Image   | `/zed/zed_node/left/image_rect_color`        |
| Depth Image | `/zed/zed_node/depth/depth_registered`       |
| Camera Info | `/zed/zed_node/left/camera_info`             |
| IMU         | `/zed/zed_node/imu/data`                     |
| Pose / Odom | `/zed/zed_node/odom`                         |
| Point Cloud | `/zed/zed_node/point_cloud/cloud_registered` |

---

## Repository Structure

```text
tools_zed2i_m/
├── @ZED2i/                 % MATLAB class implementation
│   ├── ZED2i.m
│   ├── cfgParameters.m
│   ├── cfgState.m
│   ├── lcConnect.m
│   ├── lcDisconnect.m
│   ├── rGrab.m
│   ├── getImage.m
│   ├── getDepth.m
│   ├── getCalibration.m
│   ├── getIntrinsics.m
│   ├── getImu.m
│   ├── getPose.m
│   ├── getPointCloud.m
│   └── getSensorData.m
│
├── Examples/               % Usage and validation demos
│   ├── demo_zed2i_calibration.m
│   ├── demo_zed2i_rgbd.m
│   ├── demo_zed2i_state.m
│   └── demo_zed2i_pointcloud.m
│
├── README.md
└── LICENSE
```

---

## Usage

### 1. Minimal RGB Acquisition

```matlab
zed = ZED2i();
zed.lcConnect();

for k = 1:100
    if zed.rGrab()
        imshow(zed.getImage());
        drawnow;
    end
end

zed.lcDisconnect();
```

---

### 2. Aggregated Access (Recommended)

The recommended way to access data is via `getSensorData`, which returns a
**stable snapshot struct** aggregating all enabled sensors.

```matlab
zed = ZED2i("enableImu", true, "enablePose", true);
zed.lcConnect();

data = zed.getSensorData();

imshow(data.Image);
disp(data.Metrics);

zed.lcDisconnect();
```

Returned fields may include:

* `Image`
* `Depth`
* `Imu`
* `Pose`
* `PointCloud`
* `Calibration`
* `Metrics`
* `Timestamp`
* `Connected`
* `LastError`

Availability is indicated by `HasImage`, `HasDepth`, `HasImu`, `HasPose`,
`HasPointCloud`.

---

### 3. RGB-D Visualization

```matlab
run Examples/demo_zed2i_rgbd.m
```

Provides a real-time RGB-D preview with:

* FPS estimation
* drop counters
* adaptive depth contrast

---

### 4. IMU + Pose State Demo

```matlab
run Examples/demo_zed2i_state.m
```

Demonstrates:

* IMU angular velocity and linear acceleration
* Pose position and orientation (Euler angles)
* Trajectory plotting
* Time-series plots of motion quantities

---

### 5. Point Cloud Visualization

```matlab
run Examples/demo_zed2i_pointcloud.m
```

Features:

* Snapshot or real-time point cloud acquisition
* Subsampling for performance
* Fixed spatial scale (e.g., 4×4×4 m cube)
* Safe visualization with `pcshow`

---

## Notes and Design Decisions

* Depth values are provided **in meters** (`single` precision).
* Invalid depth values are represented as `NaN`.
* Calibration data is fetched **once** and cached.
* Optional sensors are enabled explicitly to reduce overhead.
* MATLAB does **not** modify ROS parameters.
* All data is consumed **exactly as published** by the ROS 2 ZED node.
* Resolution, depth mode, and filters must be configured on the ROS side.

---

## Intended Use

This wrapper is intended for:

* robotics laboratory classes,
* perception and mapping experiments,
* MATLAB-based prototyping,
* academic research and teaching.

It is **not** intended to replace the ZED SDK or ROS-side processing pipelines.

---

## License

This project is distributed under the terms described in the `LICENSE` file.
