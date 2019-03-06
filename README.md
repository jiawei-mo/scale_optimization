# We extend SVO to a stereo system using scale optimization
Copyright (c) <2019> <Jiawei Mo, Junaed Sattar>

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

# Note
Make sure SVO works in your case since it is very buggy

# Dependencies
[SVO](https://github.com/uzh-rpg/rpg_svo)

# Installation
```
cd ~/catkin_ws/src
git clone https://github.com/jiawei-mo/scale_optimization.git
git checkout so_svo
cd ~/catkin_ws
catkin_make
```

# Usage
- Calibrate stereo cameras using [Kalibr](https://github.com/ethz-asl/kalibr)

- Create a launch file by the following the launch files in launch directory

```
roslaunch so_svo [YOUR_LAUNCH_FILE]
```
