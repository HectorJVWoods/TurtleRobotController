#!/bin/bash

colcon build --packages-select py_pubsub
source install/setup.bash
ros2 run py_pubsub talker
