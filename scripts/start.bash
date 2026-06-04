#!/usr/bin/env bash

source /opt/ros/jazzy/setup.bash
[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash
[ -f /ws/.venv/bin/activate ] && source /ws/.venv/bin/activate

colcon build

sleep infinity
