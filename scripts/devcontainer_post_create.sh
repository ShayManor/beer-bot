#!/usr/bin/env bash
set -euo pipefail

: "${ROS_DISTRO:=jazzy}"

apt update
apt install python3.12-venv

if [ -d "/ws/.venv" ]; then
    rm -rf /ws/.venv
fi
python3 -m venv /ws/.venv --system-site-packages
source /ws/.venv/bin/activate
pip install -r /ws/requirements.txt

# Add ROS2 python packages to interpreter to make life easier
cat > /ws/.venv/lib/python3.12/site-packages/ros2.pth << EOF
/opt/ros/jazzy/lib/python3.12/site-packages
/opt/ros/jazzy/local/lib/python3.12/dist-packages
EOF

# Source ROS and prep workspace
if [ -f "/opt/ros/jazzy/setup.bash" ]; then
  source "/opt/ros/jazzy/setup.bash"
fi

if [ ! -d /ws/.venv ]; then
  python3 -m venv /ws/.venv
fi
source /ws/.venv/bin/activate
python -m pip install --upgrade pip setuptools wheel

if [ -f /ws/requirements.txt ]; then
  pip install -r /ws/requirements.txt
fi
if [ -f /ws/requirements.local.txt ]; then
  pip install -r /ws/requirements.local.txt || true
fi

rosdep update || true
rosdep install --from-paths /ws/src --ignore-src -y || true

colcon build --symlink-install || true
