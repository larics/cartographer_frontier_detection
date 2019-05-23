## Freiburg FR-079 dataset

To run:

```
gunzip fr079-original-uncorrected.bag.gz
roscore &
rosparam set use_sim_time true
rosrun cartographer_ros cartographer_offline_node -configuration_directory . \
    -configuration_basenames fr079.lua -bag_filenames fr079-original-uncorrected.bag \
    odom:=ODOM scan:=FLASER -use_bag_transforms=false
rviz -d fr079.rviz
```

### Source

Obtained from https://www.mrpt.org/Dataset_fr079. Data set recorded by Cyrill Stachniss in Building 079 at the University of Freiburg.
