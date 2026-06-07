# Autonomous Rover

[![tests](../../actions/workflows/tests.yml/badge.svg)](../../actions/workflows/tests.yml)

## Build

```bash
colcon build --packages-select autonomous_rover
source install/setup.bash
colcon test --packages-select autonomous_rover
```

## Run

```bash
ros2 launch autonomous_rover bringup_sim.launch.py   # simulation
ros2 launch autonomous_rover bringup_pi.launch.py    # on-device
```
