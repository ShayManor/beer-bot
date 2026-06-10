"""One-time: dump a cached QNN context binary for the Rubik Pi 3 (QCS6490) NPU.

Run on the Pi after fetching the fp32 ONNX, then point depth_model_path at --out.
"""
import argparse

from autonomous_rover.nodes.localization.depth import make_session, parse_qnn_options


def main(argv=None):
    ap = argparse.ArgumentParser(description="Compile a QNN context binary (.onnx).")
    ap.add_argument("--model", required=True, help="fp32 source .onnx")
    ap.add_argument("--out", required=True, help="output *_ctx.onnx path")
    ap.add_argument("--options", nargs="*", default=[],
                    help='QNN provider options as k=v (e.g. backend_path=libQnnHtp.so)')
    args = ap.parse_args(argv)

    opts = parse_qnn_options(args.options)
    make_session(args.model,
                 providers=["QNNExecutionProvider", "CPUExecutionProvider"],
                 provider_options=[opts, {}],
                 compile_ctx=True, ctx_path=args.out)
    print(f"wrote {args.out}")


if __name__ == "__main__":
    main()
