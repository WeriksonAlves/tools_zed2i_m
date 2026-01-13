# ZED2i MATLAB ROS2 Wrapper

Minimal MATLAB wrapper for the ZED2i stereo camera using ROS 2,
following the laboratory class architecture pattern (`@Class` folders).

## Overview

This repository provides a minimal, stable interface to acquire RGB images
from a ZED2i camera through ROS 2 using MATLAB (R2025a).

The implementation follows the same architectural conventions used by
the robotics laboratory sensor and robot classes.

## Requirements

- MATLAB R2025a
- Robotics System Toolbox (with ROS 2 support)
- Running ZED ROS 2 wrapper
- Active ROS 2 environment

## ROS 2 Topics Used

- `/zed/zed_node/left/image_rect_color`

## Usage

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
