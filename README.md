# Frontier detection for Google Cartographer

This repository contains a version of [Google Cartographer](https://github.com/googlecartographer/cartographer) modified to implement frontier detection according to the method described in the paper "Efficient Dense Frontier Detection for 2D Graph SLAM Based on Occupancy Grid Submaps".

### Usage

To use, you can follow the usual intructions for compiling Cartographer, available [here](https://google-cartographer-ros.readthedocs.io/en/latest/compilation.html). However, instead of calling `wstool` to clone the official upstream version of Cartographer, clone this repository into your Catkin workspace instead (`git clone https://github.com/larics/cartographer_frontier_detection`). Afterwards, use the usual command for building the Catkin workspace (`catkin_make_isolated --install --use-ninja`). We strongly recommend Ubuntu 18.04 and ROS Melodic.

The code in the repository assumes you have an Intel Broadwell (or better) CPU for using Eigen AVX vectorization. If you have an older CPU (or an AMD cpu), change the following line in the file: `cartographer_ros/cartographer_ros/cartographer_ros/frontier_detection.cc`
```
#pragma GCC target ("arch=broadwell")`
```
to your CPU architecture (e.g. `sandybridge`, `skylake`). The full list of CPU architectures supported by GCC is available [here](https://gcc.gnu.org/onlinedocs/gcc/x86-Options.html).

No other configuration compared to upstream Cartographer is necessary.

### Visualization of the frontier

When cartographer is running, add a marker display for the `/frontier_marker` topic.

### Subscribing to the frontier topic

This version  of Cartographer will emit frontier updates as visualization markers ([visualization_msgs/MarkerArray](http://docs.ros.org/melodic/api/visualization_msgs/html/msg/MarkerArray.html)) on the `/frontier_marker` topic. Frontier points are split into submaps using marker namespaces and are sent already projected into the global map frame. A node which subscribes to the frontier marker topic should accumulate all frontier points it receives, for all submaps. When a newer marker for the same submap (with the same namespace) arrives, the previous frontier points for that submap should be replaced with the new version.

### Demo video

https://goo.gl/62zEUy
