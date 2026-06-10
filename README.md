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

The localization node runs Depth Anything V2 Metric (Indoor, Small) via ONNX
Runtime. Default is the synthetic `stub`; switch to the real net with the
`depth_estimator` param.

On the Pi (QCS6490):

1. `pip install onnxruntime-qnn opencv-python` and ensure the QNN libs
   (`libQnnHtp.so`) are on the loader path.
2. Fetch the fp32 model into `models/` (gitignored):
   `models/depth_anything_v2_metric_indoor_small.onnx`.
3. Compile a cached context binary once (runs fp16 on the HTP NPU):
   ```
   ros2 run autonomous_rover compile_depth_qnn \
     --model models/depth_anything_v2_metric_indoor_small.onnx \
     --out   models/depth_anything_v2_metric_indoor_small_ctx.onnx \
     --options backend_path=libQnnHtp.so htp_arch=68
   ```
4. In `params/localization.yaml` set `depth_estimator: onnx`,
   `onnx_providers: ["QNNExecutionProvider", "CPUExecutionProvider"]`, and
   `depth_model_path` to the `*_ctx.onnx` (use an absolute path on-device — a
   relative path is resolved against the package share, where `models/` is not
   installed).

In the devcontainer/sim, leave `depth_estimator: stub` (no model or onnxruntime
needed), or use `onnx_providers: ["CPUExecutionProvider"]` with the fp32 model.
