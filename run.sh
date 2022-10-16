#!/bin/bash

echo "catkin build"
catkin_make -DCMAKE_BUILD_TYPE=Debug

echo "source devel loc"
source devel/setup.bash

echo "rosrun"
rosrun map_segmentation map_segmentation_mapSegmentation

