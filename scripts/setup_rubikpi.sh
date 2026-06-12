#!/usr/bin/env bash
# =============================================================================
# setup_rubikpi.sh — provision a Rubik Pi 3 (Qualcomm QCS6490) to run the
# Depth-Anything-V2 INT8 depth net on the Hexagon HTP NPU via onnxruntime-QNN.
#
# The stock ubuntu-qcom-iot image ships the kernel FastRPC driver + cDSP
# firmware but NONE of the userspace needed to reach the NPU. This script lays
# down the full chain we reverse-engineered:
#   libcdsprpc.so (FastRPC userspace)  ->  /dev/fastrpc-cdsp permission
#   ->  fastrpc_shell_unsigned_3 (dynamic-PD shell)  ->  libQnnHtpV68Skel.so
#   ->  the skel's libc++ Hexagon runtime deps
# then verifies every link, ending with a real on-NPU inference.
#
# Idempotent: safe to re-run. Override any path via the env vars below.
#
# Usage:   bash scripts/setup_rubikpi.sh            # provision + test
#          SKIP_BUILD=1 bash scripts/setup_rubikpi.sh   # don't rebuild libcdsprpc
#          bash scripts/setup_rubikpi.sh --test-only    # only run the checks
# =============================================================================
set -uo pipefail

# ---- config (override via environment) --------------------------------------
QAIRT_ROOT="${QAIRT_ROOT:-}"                       # auto-detected if empty
DSP_ARCH="${DSP_ARCH:-v68}"                        # QCS6490 == Hexagon v68
RFSA_DIR="${RFSA_DIR:-/usr/lib/rfsa/adsp}"         # cDSP file-relay search dir
LIBDIR="${LIBDIR:-/usr/lib/aarch64-linux-gnu}"     # where libcdsprpc.so lands
ENV_FILE="${ENV_FILE:-/etc/profile.d/qnn-rover.sh}"
UDEV_RULE="${UDEV_RULE:-/etc/udev/rules.d/91-fastrpc.rules}"
FASTRPC_SRC="${FASTRPC_SRC:-$HOME/fastrpc}"        # quic/fastrpc checkout
MODELS_DIR="${MODELS_DIR:-$HOME/autonomous-rover/models}"
MODEL="${MODEL:-depth_anything_v2_metric_indoor_small_gelu_int8_252.onnx}"
BOARD_DSP="${BOARD_DSP:-}"                          # board's cdsp dir, auto-detected

# ---- pretty logging ---------------------------------------------------------
if [ -t 1 ]; then R=$'\e[31m'; G=$'\e[32m'; Y=$'\e[33m'; B=$'\e[36m'; N=$'\e[0m'; else R=; G=; Y=; B=; N=; fi
say()  { printf '%s==>%s %s\n' "$B" "$N" "$*"; }
ok()   { printf '%s  ok%s %s\n' "$G" "$N" "$*"; }
warn() { printf '%swarn%s %s\n' "$Y" "$N" "$*"; }
die()  { printf '%sFAIL%s %s\n' "$R" "$N" "$*" >&2; exit 1; }
PASS=0; FAILED=0
check(){ if eval "$2" >/dev/null 2>&1; then printf '%s PASS%s %s\n' "$G" "$N" "$1"; PASS=$((PASS+1)); else printf '%s FAIL%s %s\n' "$R" "$N" "$1"; FAILED=$((FAILED+1)); return 1; fi; }

SUDO=""; [ "$(id -u)" -ne 0 ] && SUDO="sudo"
TEST_ONLY=0; [ "${1:-}" = "--test-only" ] && TEST_ONLY=1

# =============================================================================
# Preflight — detect the board, the SoC, and the QAIRT SDK
# =============================================================================
preflight() {
  say "Preflight"
  [ "$(uname -m)" = "aarch64" ] || die "not aarch64 (got $(uname -m)) — run this on the Rubik Pi"
  local model; model="$(tr -d '\0' < /sys/firmware/devicetree/base/model 2>/dev/null || true)"
  case "$model" in
    *RUBIK*|*Qualcomm*|*QCS6490*|*qcm6490*) ok "board: ${model:-unknown}";;
    *) warn "unrecognized board model: '${model:-?}' — continuing anyway";;
  esac
  [ -c /dev/fastrpc-cdsp ] || die "/dev/fastrpc-cdsp missing — the kernel FastRPC driver / cDSP firmware isn't up (re-flash the Qualcomm image)"
  ok "cDSP device node present"

  # locate QAIRT (prefer 2.35.x to match the onnxruntime-qnn 1.23 wheel)
  if [ -z "$QAIRT_ROOT" ]; then
    QAIRT_ROOT="$(ls -d /home/*/qairt/2.35.* /opt/qairt/2.35.* 2>/dev/null | head -1)"
    [ -z "$QAIRT_ROOT" ] && QAIRT_ROOT="$(ls -d /home/*/qairt/* /opt/qairt/* 2>/dev/null | head -1)"
  fi
  [ -n "$QAIRT_ROOT" ] && [ -d "$QAIRT_ROOT" ] || die "QAIRT SDK not found — set QAIRT_ROOT=/path/to/qairt/<ver>"
  # Prefer the Ubuntu build (matches this OS); fall back to OpenEmbedded only if absent.
  QAIRT_AARCH="$(ls -d "$QAIRT_ROOT"/lib/aarch64-ubuntu-* 2>/dev/null | head -1)"
  [ -z "$QAIRT_AARCH" ] && QAIRT_AARCH="$(ls -d "$QAIRT_ROOT"/lib/aarch64-oe-* 2>/dev/null | head -1)"
  QAIRT_HEX="$QAIRT_ROOT/lib/hexagon-$DSP_ARCH/unsigned"
  [ -f "$QAIRT_AARCH/libQnnHtp.so" ] || die "libQnnHtp.so not under $QAIRT_AARCH"
  [ -f "$QAIRT_HEX/libQnnHtp${DSP_ARCH^}Skel.so" ] || die "skel not under $QAIRT_HEX"
  ok "QAIRT: $QAIRT_ROOT"
  ok "  aarch64 libs: $QAIRT_AARCH"
  ok "  hexagon skels: $QAIRT_HEX"

  # locate the board's cDSP dir (matched shells + Hexagon libc++)
  if [ -z "$BOARD_DSP" ]; then
    BOARD_DSP="$(dirname "$(ls /usr/share/qcom/qcm6490/*/*/dsp/cdsp/fastrpc_shell_unsigned_3 2>/dev/null | head -1)" 2>/dev/null)"
    [ -z "$BOARD_DSP" ] && BOARD_DSP="$(dirname "$(find /usr/share/qcom -path '*/cdsp/fastrpc_shell_unsigned_3' 2>/dev/null | head -1)" 2>/dev/null)"
  fi
  [ -n "$BOARD_DSP" ] && [ -d "$BOARD_DSP" ] || die "board cDSP dir (fastrpc_shell_unsigned_3 + libc++) not found — set BOARD_DSP=..."
  ok "board cDSP files: $BOARD_DSP"
  SKEL="libQnnHtp${DSP_ARCH^}Skel.so"
}

# =============================================================================
# Stage 1 — FastRPC userspace: build libcdsprpc.so from quic/fastrpc if absent
# =============================================================================
stage_fastrpc() {
  say "Stage 1: FastRPC userspace (libcdsprpc.so)"
  if ldconfig -p | grep -q 'libcdsprpc\.so'; then ok "libcdsprpc already installed"; return; fi
  [ "${SKIP_BUILD:-0}" = "1" ] && die "libcdsprpc.so missing and SKIP_BUILD=1"
  say "building from quic/fastrpc (one-time)…"
  $SUDO apt-get update -qq || warn "apt update failed (offline?) — proceeding"
  $SUDO apt-get install -y -qq build-essential automake libtool autoconf git libbsd-dev libyaml-dev \
    || die "could not install build deps"
  [ -d "$FASTRPC_SRC/.git" ] || git clone --depth 1 https://github.com/quic/fastrpc "$FASTRPC_SRC" || die "git clone failed"
  ( cd "$FASTRPC_SRC" && ./gitcompile ) || die "fastrpc build failed — see $FASTRPC_SRC"
  local built; built="$(find "$FASTRPC_SRC" -name 'libcdsprpc.so*' -type f | head -1)"
  [ -n "$built" ] || die "build produced no libcdsprpc.so"
  $SUDO cp -v "$built" "$LIBDIR/libcdsprpc.so"
  $SUDO ln -sf libcdsprpc.so "$LIBDIR/libcdsprpc.so.1"
  $SUDO ldconfig
  ldconfig -p | grep -q 'libcdsprpc\.so' || die "libcdsprpc still not visible to ldconfig"
  ok "libcdsprpc.so installed"
}

# =============================================================================
# Stage 2 — make /dev/fastrpc-cdsp usable by non-root (persistent udev rule)
# =============================================================================
stage_udev() {
  say "Stage 2: /dev/fastrpc-cdsp permissions (udev)"
  if [ ! -f "$UDEV_RULE" ]; then
    echo 'KERNEL=="fastrpc-cdsp*", MODE="0666"' | $SUDO tee "$UDEV_RULE" >/dev/null
    $SUDO udevadm control --reload-rules && $SUDO udevadm trigger || warn "udev reload failed"
    ok "installed udev rule $UDEV_RULE"
  else ok "udev rule already present"; fi
  $SUDO chmod a+rw /dev/fastrpc-cdsp /dev/fastrpc-cdsp-secure 2>/dev/null || true   # take effect now
}

# =============================================================================
# Stage 3 — populate the cDSP file-relay dir: shells + HTP skel + libc++ deps
# (The HTP skel NEEDs libc++.so.1/libc++abi.so.1 — the hidden blocker.)
# =============================================================================
stage_rfsa() {
  say "Stage 3: DSP runtime files -> $RFSA_DIR"
  $SUDO mkdir -p "$RFSA_DIR"
  # dynamic-PD shells (board-matched, signed + unsigned) + adsp shell
  for f in fastrpc_shell_3 fastrpc_shell_unsigned_3 fastrpc_shell_0; do
    [ -f "$BOARD_DSP/$f" ] && $SUDO cp -u "$BOARD_DSP/$f" "$RFSA_DIR/" && ok "shell $f"
  done
  # HTP graph skel (from QAIRT, version-matched to libQnnHtp.so)
  $SUDO cp -u "$QAIRT_HEX/$SKEL" "$RFSA_DIR/" && ok "skel  $SKEL"
  # the skel's NEEDED Hexagon C++ runtime (board-matched)
  for f in libc++.so.1 libc++abi.so.1; do
    [ -f "$BOARD_DSP/$f" ] || die "missing $f in $BOARD_DSP (skel cannot load without it)"
    $SUDO cp -u "$BOARD_DSP/$f" "$RFSA_DIR/" && ok "dep   $f"
  done
}

# =============================================================================
# Stage 4 — environment so libQnnHtp.so finds its companions + the DSP dir
# =============================================================================
stage_env() {
  say "Stage 4: environment ($ENV_FILE)"
  $SUDO tee "$ENV_FILE" >/dev/null <<EOF
# Auto-generated by setup_rubikpi.sh — QNN/HTP runtime paths for the depth net.
export ADSP_LIBRARY_PATH="$RFSA_DIR"
case ":\${LD_LIBRARY_PATH:-}:" in *":$QAIRT_AARCH:"*) ;; *) export LD_LIBRARY_PATH="\${LD_LIBRARY_PATH:+\$LD_LIBRARY_PATH:}$QAIRT_AARCH";; esac
EOF
  ok "wrote $ENV_FILE (the ROS launch also sets these via additional_env)"
  # export into this shell for the tests below
  export ADSP_LIBRARY_PATH="$RFSA_DIR"
  export LD_LIBRARY_PATH="${LD_LIBRARY_PATH:+$LD_LIBRARY_PATH:}$QAIRT_AARCH"
}

# =============================================================================
# Tests — verify every link, ending with a real on-NPU inference
# =============================================================================
run_tests() {
  say "Tests"
  export ADSP_LIBRARY_PATH="$RFSA_DIR"
  case ":${LD_LIBRARY_PATH:-}:" in *":$QAIRT_AARCH:"*) ;; *) export LD_LIBRARY_PATH="${LD_LIBRARY_PATH:+$LD_LIBRARY_PATH:}$QAIRT_AARCH";; esac

  check "FastRPC userspace (libcdsprpc.so) on loader path" "ldconfig -p | grep -q 'libcdsprpc\.so'"
  check "/dev/fastrpc-cdsp accessible (rw)"               "test -r /dev/fastrpc-cdsp && test -w /dev/fastrpc-cdsp"
  check "dynamic-PD shell present"                        "test -f '$RFSA_DIR/fastrpc_shell_unsigned_3'"
  check "HTP skel present"                                "test -f '$RFSA_DIR/$SKEL'"
  check "skel libc++ deps present"                        "test -f '$RFSA_DIR/libc++.so.1' && test -f '$RFSA_DIR/libc++abi.so.1'"
  check "cDSP remoteproc running"                         "grep -qi running /sys/class/remoteproc/*/state 2>/dev/null"

  # Qualcomm's own HTP self-test (hardware path, independent of onnxruntime)
  local PV; PV="$(find "$QAIRT_ROOT/bin" -name qnn-platform-validator -path '*aarch64-ubuntu*' 2>/dev/null | head -1)"
  [ -z "$PV" ] && PV="$(find "$QAIRT_ROOT/bin" -name qnn-platform-validator -path '*aarch64-oe*' 2>/dev/null | head -1)"
  if [ -n "$PV" ]; then
    # NB: the validator prints results to stderr and exits non-zero on a cosmetic
    # "Error in saving the results" — so merge streams and match on the text, not $?.
    mkdir -p /tmp/pv 2>/dev/null || true
    if "$PV" --backend dsp --testBackend --targetPath /tmp/pv 2>&1 | grep -q "Unit Test on the backend DSP: Passed"; then
      printf '%s PASS%s qnn-platform-validator: DSP unit test\n' "$G" "$N"; PASS=$((PASS+1))
    else
      printf '%s FAIL%s qnn-platform-validator: DSP unit test\n' "$R" "$N"; FAILED=$((FAILED+1))
    fi
  else warn "qnn-platform-validator not found; skipping HTP self-test"; fi

  # onnxruntime QNN EP availability
  if python3 -c "import onnxruntime as o; exit(0 if 'QNNExecutionProvider' in o.get_available_providers() else 1)" 2>/dev/null; then
    printf '%s PASS%s onnxruntime QNNExecutionProvider available\n' "$G" "$N"; PASS=$((PASS+1))
  else
    printf '%s FAIL%s onnxruntime QNNExecutionProvider missing — install the aarch64 onnxruntime-qnn wheel\n' "$R" "$N"; FAILED=$((FAILED+1))
  fi

  # End-to-end: load the depth model on the NPU, confirm 1 partition + latency
  local MP="$MODELS_DIR/$MODEL"
  if [ -f "$MP" ] && python3 -c "import onnxruntime,numpy" 2>/dev/null; then
    say "End-to-end NPU inference ($MODEL)"
    ADSP_LIBRARY_PATH="$RFSA_DIR" QAIRT_AARCH="$QAIRT_AARCH" MP="$MP" python3 - <<'PY'
import os, time, numpy as np, onnxruntime as ort
ort.set_default_logger_severity(3)
mp=os.environ["MP"]; bp=os.path.join(os.environ["QAIRT_AARCH"],"libQnnHtp.so")
opts={"backend_path":bp,"htp_performance_mode":"burst","htp_graph_finalization_optimization_mode":"3"}
so=ort.SessionOptions(); so.log_severity_level=3
try:
    s=ort.InferenceSession(mp,sess_options=so,providers=["QNNExecutionProvider","CPUExecutionProvider"],provider_options=[opts,{}])
except Exception as e:
    print("\033[31m FAIL\033[0m session create:",repr(e)[:160]); raise SystemExit(1)
inp=s.get_inputs()[0]; shp=[d if isinstance(d,int) else 1 for d in inp.shape]
x=np.random.rand(*shp).astype(np.float32)
for _ in range(3): s.run(None,{inp.name:x})
t=time.time()
for _ in range(10): y=s.run(None,{inp.name:x})[0]
ms=(time.time()-t)/10*1000
provs=s.get_providers()
# 252x252 is ~33 ms on the NPU vs ~340 ms on CPU — 150 ms cleanly separates them.
fast = ("QNNExecutionProvider" in provs) and ms < 150
tag = "\033[32m PASS\033[0m" if fast else "\033[31m FAIL\033[0m"
print(f"{tag} NPU inference {shp} -> {y.shape} in {ms:.0f} ms/frame  (providers={provs})")
if not fast: print("       ^ too slow => CPU fallback, NPU not engaged; check the failures above")
raise SystemExit(0 if fast else 1)
PY
    [ $? -eq 0 ] && PASS=$((PASS+1)) || FAILED=$((FAILED+1))
  else
    warn "model $MP or onnxruntime/numpy missing — skipping end-to-end inference test"
  fi

  echo
  if [ "$FAILED" -eq 0 ]; then printf '%s==> ALL %d CHECKS PASSED%s — the NPU depth net is ready.\n' "$G" "$PASS" "$N"
  else printf '%s==> %d passed, %d FAILED%s — see above.\n' "$Y" "$PASS" "$FAILED" "$N"; exit 1; fi
}

# ---- main -------------------------------------------------------------------
preflight
if [ "$TEST_ONLY" -eq 0 ]; then
  stage_fastrpc
  stage_udev
  stage_rfsa
  stage_env
fi
run_tests
