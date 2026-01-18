# ZED2i MATLAB ROS 2 Wrapper

MATLAB wrapper for the **ZED2i stereo camera** using **ROS 2**, following the
robotics laboratory class architecture pattern (`@Class` folders).

This repository provides a **stable, minimal, and extensible interface**
to access RGB images, depth maps, and camera calibration data from a ZED2i
camera inside MATLAB.

---

## Overview

This project implements a MATLAB interface to the **ZED2i** camera through
the official **ZED ROS 2 wrapper**, targeting **experimental and educational
use in robotics laboratories**.

The design follows the same architectural conventions used by existing
laboratory sensor and robot classes, enabling:

* consistent APIs across sensors,
* predictable lifecycle management,
* safe integration into control, perception, and mapping pipelines.

The implementation has been tested with **MATLAB R2025a**.

---

## Features

* RGB image acquisition
* Depth image acquisition (metric, in meters)
* Camera calibration retrieval via ROS 2 `CameraInfo`
* Lazy caching of calibration parameters
* Aggregated, lab-style access via `rGetSensorData`
* Optional conversion to MATLAB `cameraIntrinsics`
* Efficient real-time visualization (RGB-D)
* Minimal, modular, and extensible class design

---

## Requirements

### MATLAB

* MATLAB **R2025a**
* **Robotics System Toolbox** (ROS 2 support required)
* *(Optional)* **Computer Vision Toolbox**
  Required only for the `cameraIntrinsics` helper

### ROS 2 and ZED

* ROS 2 environment properly sourced
* ZED ROS 2 wrapper installed and running
* A ZED2i camera connected and detected by the ZED SDK

> ⚠️ This wrapper **does not start the ZED node**.
> The ZED ROS 2 wrapper must already be running and publishing topics.

---

## ZED ROS 2 Wrapper (Expected Setup)

Before using this MATLAB wrapper, ensure that the ZED ROS 2 node is active.

Typical launch example (outside MATLAB):

```bash
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2i
```

You should see topics similar to:

```bash
/zed/zed_node/left/image_rect_color
/zed/zed_node/depth/depth_registered
/zed/zed_node/left/camera_info
```

These topics are **consumed directly** by the MATLAB class.

---

## ROS 2 Topics Used

Default topics (configurable via constructor):

* RGB image
  `/zed/zed_node/left/image_rect_color`
* Depth image
  `/zed/zed_node/depth/depth_registered`
* Camera calibration
  `/zed/zed_node/left/camera_info`

---

## Repository Structure

```text
tools_zed2i_m/
├── @ZED2i/        % MATLAB class implementation
│   ├── ZED2i.m
│   ├── iParameters.m
│   ├── iControlVariables.m
│   ├── rConnect.m
│   ├── rDisconnect.m
│   ├── rGrab.m
│   ├── rGetImage.m
│   ├── rGetDepth.m
│   ├── rGetCalibration.m
│   ├── rGetIntrinsics.m
│   └── rGetSensorData.m
│
├── Examples/      % Usage and validation examples
│   ├── demo_zed2i_calibration.m
│   └── demo_zed2i_rgbd.m
│
├── README.md
└── LICENSE
```

---

## Usage

### 1. Minimal RGB Acquisition

```matlab
zed = ZED2i();
zed.rConnect();

for k = 1:100
    if zed.rGrab()
        imshow(zed.rGetImage());
        drawnow;
    end
end

zed.rDisconnect();
```

---

### 2. Aggregated Access (Recommended)

The recommended way to access data is via `rGetSensorData`, which returns
a **snapshot struct** containing all relevant information.

```matlab
zed = ZED2i();
zed.rConnect();

data = zed.rGetSensorData();

imshow(data.Image);
disp(data.Metrics);

zed.rDisconnect();
```

Returned fields include:

* `Image` (RGB)
* `Depth` (meters, `single`, NaN for invalid)
* `Calibration`
* `Metrics` (FPS, drops)
* `Flags` (connection and availability)
* `Timestamp`
* `LastError`

---

### 3. RGB-D Visualization

A real-time RGB-D preview with stable performance:

```matlab
run Examples/demo_zed2i_rgbd.m
```

---

### 4. Camera Calibration

Retrieve and inspect camera calibration directly from ROS 2:

```matlab
run Examples/demo_zed2i_calibration.m
```

This uses `sensor_msgs/CameraInfo` and caches results internally.

---

## Notes and Design Decisions

* Depth values are provided **in meters** (`single` precision).
* Invalid depth values are represented as `NaN`.
* Calibration data is fetched **once** and cached.
* No ROS parameters are modified by MATLAB.
* The wrapper consumes data **exactly as published** by the ZED ROS 2 node.
* Resolution, depth mode, and confidence thresholds must be configured
  in the ZED ROS 2 wrapper.

---

## Intended Use

This wrapper is intended for:

* robotics laboratory experiments,
* perception and mapping pipelines,
* prototyping algorithms in MATLAB,
* educational and research activities.

It is **not** intended to replace the ZED SDK or ROS-side processing.

---

## License

This project is distributed under the terms described in the `LICENSE` file.
