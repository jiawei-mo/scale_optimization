# Extending Monocular Visual Odometry to Stereo Camera Systems by Scale Optimization
Copyright (c) <2019> <Jiawei Mo, Junaed Sattar>

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

# Introduction
This is the implementation of [Extending Monocular Visual Odometry to Stereo Camera Systems by Scale Optimization](https://arxiv.org/abs/1905.12723). Here, we adapted [DSO](https://github.com/JakobEngel/dso) as our underlying monocular visual odometry.

# Dependencies
[DSO](https://github.com/JakobEngel/dso)

# Installation
```
install [DSO](https://github.com/JakobEngel/dso)
export DSO_PATH=[PATH_TO_DSO]/dso

cd ~/catkin_ws/src
git clone https://github.com/jiawei-mo/scale_optimization.git
cd ~/catkin_ws
catkin_make
```

# Usage
- Calibrate stereo cameras, using the format of examples in **cams**. Refer to [DSO](https://github.com/JakobEngel/dso) for more details of intrinsic parameters.

- Create a launch file by the format of **launch/sample.launch**.

```
roslaunch so_dso [YOUR_LAUNCH_FILE]
```

- Ctrl-C to terminate and the output files will be written to **.ros** folder.

# Output files
- **poses.txt**: poses of all frame, using the TUM RGB-D / TUM monoVO format ([timestamp x y z qx qy qz qw] of the cameraToWorld transformation).

Time logs:
- **fps_time.txt**: runtime of each frame.
- **scale_time.txt**: runtime of scale optimization.
- **ba_time.txt**: runtime of bundle adjustment.

# Parameters
The only extra parameter of this work over [DSO](https://github.com/JakobEngel/dso) is the **scale_accept_th**, which is the threshold to accept a result from scaler optimizer. Usually, it is around 10-15.

# Code Explaination
- The core of scale optimization is implemented in **src/ScaleOptimizer**; **src/ScaleAccumulator** handles SSE operations in scale optimization. 
- We augmented [FullSystem](https://github.com/JakobEngel/dso/tree/master/src/FullSystem) of DSO to **src/SODSOSystem** with scale optimization. 
- The files in **src/dso_helpers** are helpers for DSO which are directly copied from DSO. 
- The main() locates in **src/so_dso_node.cpp**.