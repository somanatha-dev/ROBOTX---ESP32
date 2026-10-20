#!/usr/bin/env python3
"""Build and run every host-side test. No ESP32, no Pi, no motors.

    python tests/run_host_tests.py [--cxx g++] [--build-dir DIR]

1. protocol unit tests   -- protocol.cpp compiled on the host
2. link tests (4 runs)   -- the REAL sketch compiled into a host simulator
                            (tests/host/sim_hw.cpp) and driven over stdin/stdout:
     a. default configuration     (PCA9685 unconfirmed -> motion gated, as today)
     b. SIM_DRIVE_AVAILABLE=1     (applied values, deadband, MOTORTEST timing,
                                   and -- via SIM_FRONT_CM_FILE -- an obstacle that
                                   appears and clears at runtime: no auto-resume;
                                   via SIM_PCA_LOG_FILE -- no motor output from a
                                   partially-blocked mixed-direction DRIVE)
     c. drive available + front obstacle at 20 cm (safety-gated ACKs)
        (b and c with three valid rear sensors, SIM_REAR_TOF=1 SIM_REAR_BAD_CH=-1:
        reverse needs real rear sensing since RX-05; run a keeps the
        unconfigured rear, where reverse is refused)
     d. drive available + SIM_REAR_TOF=1: rear sensing up with the ch1 sensor
        failed and its init() blocking 5 s -- tests 27 and 28 only (RX-01: the
        loop, ACKs, TELEMETRY and the watchdog keep running; reverse stays
        blocked; TOFTEST is refused under motion control)
     The simulator is stamped by tools/gen_fw_build_id.py (header written into
     the build directory), and test 29 checks READY / DIAG SYSTEM report it.
3. rear ToF recovery    -- rear_tof.cpp + safety.cpp + tca9548a.cpp +
                           rover_i2c.cpp against a fake TCA9548A and three
                           scriptable VL53L0X (tests/host/test_rear_tof.cpp),
                           built twice: the shipped configuration
                           (REAR_TOF_RUNTIME_REINIT 0) and the kept-but-unused
                           runtime re-init path (-DREAR_TOF_RUNTIME_REINIT=1)
4. GPS bounded polling  -- gps.cpp + rover_i2c.cpp + protocol.cpp against a
                           fake u-blox DDC port and the SparkFun library stub
                           (tests/host/test_gps.cpp)
5. motion safety         -- the REAL sketch in-process on a manual clock with a
                           GPIO + PCA9685 event log (tests/host/test_motion_safety.cpp):
                           GPIO-first order in every stop path, reverse policy,
                           TOFTEST refusal, build identity. Built twice: stamped
                           with -DFW_BUILD_ID, and unstamped ("unversioned")
6. VL53L0X VCSEL guard   -- the REAL installed Pololu library (--vl53-lib,
                           default: the sketchbook's libraries/VL53L0X) against
                           a fake sensor whose VCSEL period read fails
                           (tests/host/test_vl53_guard.cpp). An unpatched
                           library crashes this test with a divide-by-zero.
7. PCA9685 recovery      -- the REAL sketch AND the real pca9685.cpp (every other
                           suite stubs it) against a register-level fake PCA9685
                           on a manual clock (tests/host/test_pca_recovery.cpp):
                           boot NOT_FOUND / INIT_FAILED, read-back check,
                           transient / persistent / hung-bus errors, backoff,
                           and that no recovery energises a wheel, resumes an
                           old request or MOTORTEST, or clears a latch.

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
            "tca9548a.cpp", "rover_i2c.cpp", "gps.cpp"]

REAR_TOF = ["rear_tof.cpp", "safety.cpp", "tca9548a.cpp", "rover_i2c.cpp"]

GPS = ["gps.cpp", "rover_i2c.cpp", "protocol.cpp"]

# (simulator environment, extra test_link.py arguments)
SIM_RUNS = [
    (["SIM_ROM_NOISE=1"], []),
    # b and c drive in reverse, so they need rear sensing that permits it: since
    # SAFETY_BLOCK_REVERSE_WHEN_REAR_UNCONFIGURED is 1 (RX-05) an unconfigured
    # rear blocks reverse. SIM_REAR_TOF=1 with no faulty sensor = three valid,
    # clear rear sensors, as on the commissioned rover.
    (["SIM_DRIVE_AVAILABLE=1", "SIM_FRONT_CM_FILE={front_file}", "SIM_PCA_LOG_FILE={pca_log}",
      "SIM_REAR_TOF=1", "SIM_REAR_BAD_CH=-1"], []),
    (["SIM_DRIVE_AVAILABLE=1", "SIM_FRONT_CM=20", "SIM_REAR_TOF=1", "SIM_REAR_BAD_CH=-1"], []),
    (["SIM_DRIVE_AVAILABLE=1", "SIM_REAR_TOF=1", "SIM_REAR_BAD_CH=1", "SIM_REAR_INIT_DELAY_MS=5000"],
     ["--only", "27,28"]),
]


def header_build_id(path):
    """FW_BUILD_ID from a generated fw_build_id.h, or None."""
    try:
        with open(path) as fh:
            for line in fh:
                if line.startswith("#define FW_BUILD_ID "):
                    return line.split('"')[1]
    except OSError:
        return None
    return None


def run(cmd):
    print("+", " ".join(cmd), flush=True)
    return subprocess.call(cmd)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cxx", default="g++")
    ap.add_argument("--build-dir", default=os.path.join(tempfile.gettempdir(), "rover_host_build"))
    ap.add_argument("--vl53-lib", default=os.path.join(os.path.dirname(ROOT), "libraries", "VL53L0X"))
    args = ap.parse_args()
    os.makedirs(args.build_dir, exist_ok=True)

    exe = ".exe" if os.name == "nt" else ""
    unit = os.path.join(args.build_dir, "test_protocol" + exe)
    sim = os.path.join(args.build_dir, "rover_sim" + exe)
    flags = ["-std=gnu++11", "-Wall", "-Wextra", "-O1"]

    if run([args.cxx, *flags, "-I", ROOT, os.path.join(HERE, "host", "test_protocol.cpp"),
            os.path.join(ROOT, "protocol.cpp"), "-o", unit]):
        return 1

    # Stamp the simulator the way a real build is stamped: the generator writes
    # fw_build_id.h, here into the build directory (never into the sketch).
    # A fw_build_id.h already in the sketch directory is found first by
    # comm.cpp's #include "...", so then THAT is what the firmware reports.
    stamp = os.path.join(args.build_dir, "fw_build_id.h")
    if run([sys.executable, os.path.join(ROOT, "tools", "gen_fw_build_id.py"), "--out", stamp]):
        return 1
    sketch_stamp = header_build_id(os.path.join(ROOT, "fw_build_id.h"))
    expect_build = sketch_stamp or header_build_id(stamp)

    if run([args.cxx, *flags, "-I", os.path.join(HERE, "host", "stubs"), "-I", ROOT,
            "-I", args.build_dir,
            os.path.join(HERE, "host", "sim_main.cpp"), os.path.join(HERE, "host", "sim_hw.cpp"),
            *[os.path.join(ROOT, f) for f in FIRMWARE], "-lwinmm", "-o", sim]):
        return 1

    motion_builds = []
    motion_variants = [("test_motion_safety",
                        ['-DFW_BUILD_ID="harness-build"', '-DFW_BUILD_UTC="harness-utc"',
                         '-DEXPECT_FW_BUILD="harness-build"'])]
    if sketch_stamp is None:
        motion_variants.append(("test_motion_safety_unstamped", ['-DEXPECT_FW_BUILD="unversioned"']))
    else:
        print(f"NOTE: {ROOT}/fw_build_id.h exists ({sketch_stamp}); the unstamped variant is skipped")
    for name, defines in motion_variants:
        exe_path = os.path.join(args.build_dir, name + exe)
        if run([args.cxx, *flags, *defines, "-I", os.path.join(HERE, "host", "stubs"), "-I", ROOT,
                os.path.join(HERE, "host", "test_motion_safety.cpp"),
                *[os.path.join(ROOT, f) for f in FIRMWARE], "-o", exe_path]):
            return 1
        motion_builds.append(exe_path)

    pca = os.path.join(args.build_dir, "test_pca_recovery" + exe)
    if run([args.cxx, *flags, '-DFW_BUILD_ID="harness-build"', '-DFW_BUILD_UTC="harness-utc"',
            "-I", os.path.join(HERE, "host", "stubs"), "-I", ROOT,
            os.path.join(HERE, "host", "test_pca_recovery.cpp"),
            *[os.path.join(ROOT, f) for f in FIRMWARE + ["pca9685.cpp"]], "-o", pca]):
        return 1

    rear_builds = []
    for name, defines in (("test_rear_tof", []),
                          ("test_rear_tof_runtime_reinit", ["-DREAR_TOF_RUNTIME_REINIT=1"])):
        rear = os.path.join(args.build_dir, name + exe)
        if run([args.cxx, *flags, *defines, "-I", os.path.join(HERE, "host", "rear_stubs"),
                "-I", os.path.join(HERE, "host", "stubs"), "-I", ROOT,
                os.path.join(HERE, "host", "test_rear_tof.cpp"),
                *[os.path.join(ROOT, f) for f in REAR_TOF], "-o", rear]):
            return 1
        rear_builds.append(rear)

    gps = os.path.join(args.build_dir, "test_gps" + exe)
    if run([args.cxx, *flags, "-I", os.path.join(HERE, "host", "stubs"), "-I", ROOT,
            os.path.join(HERE, "host", "test_gps.cpp"),
            *[os.path.join(ROOT, f) for f in GPS], "-o", gps]):
        return 1

    # The real library: its own directory first, so the stub VL53L0X.h in
    # stubs/ is NOT used here. -Dboolean=bool: the library uses Arduino's type.
    vl53 = os.path.join(args.build_dir, "test_vl53_guard" + exe)
    if run([args.cxx, *flags, "-Dboolean=bool", "-I", args.vl53_lib,
            "-I", os.path.join(HERE, "host", "stubs"),
            os.path.join(HERE, "host", "test_vl53_guard.cpp"),
            os.path.join(args.vl53_lib, "VL53L0X.cpp"), "-o", vl53]):
        return 1

    failures = 0
    failures += run([unit]) != 0
    for rear in rear_builds:
        failures += run([rear]) != 0
    for motion in motion_builds:
        failures += run([motion]) != 0
    failures += run([pca]) != 0
    failures += run([gps]) != 0
    failures += run([vl53]) != 0
    front_file = os.path.join(args.build_dir, "sim_front_cm.txt")
    pca_log = os.path.join(args.build_dir, "sim_pca_nonzero.log")
    for env, extra in SIM_RUNS:
        with open(front_file, "w") as fh:
            fh.write("150\n")
        open(pca_log, "w").close()
        cmd = [sys.executable, os.path.join(HERE, "test_link.py"), "--sim", sim]
        for kv in env:
            cmd += ["--sim-env", kv.format(front_file=front_file, pca_log=pca_log)]
        if expect_build:
            cmd += ["--expect-fw-build", expect_build]
        failures += run(cmd + extra) != 0

    print("HOST TESTS:", "ALL PASSED" if failures == 0 else f"{failures} suite(s) FAILED")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
