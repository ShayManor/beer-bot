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

### Depth net (Rubik Pi 3 NPU)

The localization node runs Depth Anything V2 Metric (Indoor, Small) as a
fixed-shape INT8 ONNX on the QCS6490 Hexagon NPU via onnxruntime-QNN — ~32 ms per
frame at 252×252 (vs ~1.4 s on CPU). The shipped model is GELU-fused and
graph-simplified so QNN runs it as a single partition.

**Device setup.** The stock Ubuntu image has the kernel FastRPC driver and cDSP
firmware but none of the userspace needed to reach the NPU. Provision and verify
a board with one command:

```bash
bash scripts/setup_rubikpi.sh          # FastRPC userspace, DSP shells, HTP skel +
                                       # libc++ deps, udev rule, env — then 9 checks
                                       # ending in a real on-NPU inference
bash scripts/setup_rubikpi.sh --test-only   # health-check an already-set-up board
```

Two large prerequisites must already be on the board (the script checks for both):
the **QAIRT 2.35.0** SDK, and the aarch64 **`onnxruntime-qnn`** wheel (the build that
actually exposes `QNNExecutionProvider`). Everything else the script installs.

The model path, input size (252), QNN backend, and the `ADSP_LIBRARY_PATH` /
`LD_LIBRARY_PATH` the node needs are already wired in `params/localization.yaml` and
`launch/bringup_pi.launch.py`; `depth_model_path` is absolute on-device.

In the devcontainer/sim, leave `depth_estimator: stub` (no model or onnxruntime
needed), or set `onnx_providers: ["CPUExecutionProvider"]` to run the same model on CPU.

