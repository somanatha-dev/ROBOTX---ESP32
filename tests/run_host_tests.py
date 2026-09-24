#!/usr/bin/env python3
"""Build and run every host-side test. No ESP32, no Pi, no motors.

    python tests/run_host_tests.py [--cxx g++] [--build-dir DIR]

1. protocol unit tests   -- protocol.cpp compiled on the host
2. link tests (3 runs)   -- the REAL sketch compiled into a host simulator
                            (tests/host/sim_hw.cpp) and driven over stdin/stdout:
     a. default configuration     (PCA9685 unconfirmed -> motion gated, as today)
     b. SIM_DRIVE_AVAILABLE=1     (applied values, deadband, MOTORTEST timing)
     c. drive available + front obstacle at 20 cm (safety-gated ACKs)

Build output goes to a temporary directory, never into the repository.
"""

import argparse
import os
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)

FIRMWARE = ["comm.cpp", "protocol.cpp", "motor.cpp", "safety.cpp", "rear_tof.cpp",
            "tca9548a.cpp", "rover_i2c.cpp"]

SIM_RUNS = [
    ["SIM_ROM_NOISE=1"],
    ["SIM_DRIVE_AVAILABLE=1"],
    ["SIM_DRIVE_AVAILABLE=1", "SIM_FRONT_CM=20"],
]


def run(cmd):
    print("+", " ".join(cmd), flush=True)
    return subprocess.call(cmd)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cxx", default="g++")
    ap.add_argument("--build-dir", default=os.path.join(tempfile.gettempdir(), "rover_host_build"))
    args = ap.parse_args()
    os.makedirs(args.build_dir, exist_ok=True)

    exe = ".exe" if os.name == "nt" else ""
    unit = os.path.join(args.build_dir, "test_protocol" + exe)
    sim = os.path.join(args.build_dir, "rover_sim" + exe)
    flags = ["-std=gnu++11", "-Wall", "-Wextra", "-O1"]

    if run([args.cxx, *flags, "-I", ROOT, os.path.join(HERE, "host", "test_protocol.cpp"),
            os.path.join(ROOT, "protocol.cpp"), "-o", unit]):
        return 1
    if run([args.cxx, *flags, "-I", os.path.join(HERE, "host", "stubs"), "-I", ROOT,
            os.path.join(HERE, "host", "sim_main.cpp"), os.path.join(HERE, "host", "sim_hw.cpp"),
            *[os.path.join(ROOT, f) for f in FIRMWARE], "-lwinmm", "-o", sim]):
        return 1

    failures = 0
    failures += run([unit]) != 0
    for env in SIM_RUNS:
        cmd = [sys.executable, os.path.join(HERE, "test_link.py"), "--sim", sim]
        for kv in env:
            cmd += ["--sim-env", kv]
        failures += run(cmd) != 0

    print("HOST TESTS:", "ALL PASSED" if failures == 0 else f"{failures} suite(s) FAILED")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
