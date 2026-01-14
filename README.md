# ZED2i MATLAB ROS2 Wrapper

MATLAB wrapper for the ZED2i stereo camera using ROS 2, following the
robotics laboratory class architecture pattern (`@Class` folders).

## Overview

This repository provides a stable and incremental MATLAB interface to the
ZED2i camera through ROS 2 (tested with MATLAB R2025a).

The implementation follows the same architectural conventions used by the
laboratory sensor and robot classes, enabling seamless integration into
existing experimental pipelines.

## Features

- RGB image stream acquisition
- Depth stream acquisition (in meters)
- Camera calibration via ROS 2 `CameraInfo`
- Aggregated access through `rGetSensorData`
- Optional MATLAB `cameraIntrinsics` helper
- Minimal, modular, and extensible design

## Requirements

- MATLAB R2025a
- Robotics System Toolbox (with ROS 2 support)
- (Optional) Computer Vision Toolbox (for `cameraIntrinsics`)
- Running ZED ROS 2 wrapper
- Active ROS 2 environment

## ROS 2 Topics Used

- `/zed/zed_node/left/image_rect_color`
- `/zed/zed_node/depth/depth_registered`
- `/zed/zed_node/left/camera_info`

## Repository Structure

```matlab
@ZED2i/        % MATLAB class implementation
Examples/      % Usage and validation examples
README.md
LICENSE
```

## Usage

### Minimal RGB acquisition

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

### Aggregated access (recommended)

```matlab
zed = ZED2i();
zed.rConnect();

data = zed.rGetSensorData();

imshow(data.Image);
disp(data.Metrics);

zed.rDisconnect();
```

## Examples

* `demo_zed2i_minimal.m` – RGB stream validation
* `demo_zed2i_rgbd.m` – RGB-D visualization
* `demo_zed2i_calibration.m` – Camera calibration (CameraInfo)
* `demo_zed2i_intrinsics.m` – MATLAB `cameraIntrinsics` helper
* `demo_zed2i_getsensordata.m` – Lab-style aggregated access

## Notes

* Depth values are provided in meters (`single` precision).
* Invalid depth values are represented as `NaN`.
* Calibration parameters are cached after the first retrieval.
* The wrapper consumes the data exactly as published by the ZED ROS 2 node;
  resolution and format are defined by the ROS configuration.

## License

This project is distributed under the terms of the LICENSE file.
