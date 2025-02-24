#!/bin/bash

echo "catkin make ..."
catkin_make > /dev/null

echo "source devel/setup.bash ..."
source devel/setup.bash > /dev/null

echo "Setup ready!"