#!/usr/bin/env python3
"""Run with python3 test/verify_imu.py after pio run -e teensy41."""
import os
from pathlib import Path
import subprocess

root = Path(__file__).resolve().parents[1]
dependency = root / ".pio/libdeps/teensy41/Teensy4 I2C/src"
if not (dependency / "i2c_driver.h").exists():
    raise SystemExit("Run pio run -e teensy41 first to install Teensy4 I2C.")
scratch = root / ".scratch"
scratch.mkdir(exist_ok=True)
binary = scratch / "imu_async_test"
subprocess.run([
    os.environ.get("CXX", "c++"), "-std=c++17", "-Wall", "-Wextra", "-Werror",
    "-fsanitize=address,undefined", "-g", f"-I{root / 'include'}", f"-I{dependency}",
    str(root / "test/imu_async.cpp"), "-o", str(binary),
], check=True, timeout=30)
subprocess.run([str(binary)], check=True, timeout=5)
