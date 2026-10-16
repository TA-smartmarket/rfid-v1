"""Run firmware logic with simulated peripherals: python test/host/run.py."""
import json
from pathlib import Path
import shutil
import subprocess
import tempfile

here = Path(__file__).resolve().parent
compiler = shutil.which("g++")
if not compiler:
    raise SystemExit("Install g++ and add it to PATH to run the host tests.")
scenarios = ["startup", "restart", "rfid_offline", "transactions", "unpaired",
             "busy_timeout", "timeout_recovery", "commands", "kg_noise",
             "saturation", "storage_failure", "wraparound", "indicators", "indicators_wrap",
             "buzzer_failure"]
with tempfile.TemporaryDirectory(prefix="rfid-tests-") as temporary:
    build = Path(temporary)
    for name in ["Arduino.h", "SPI.h", "MFRC522.h", "HX711.h", "Preferences.h"]:
        (build / name).write_text('#include "mocks.hpp"\n')
    binary = build / "firmware-tests.exe"
    subprocess.run([compiler, "-std=c++11", "-Wall", "-Wextra", "-Werror",
                    "-I", str(build), "-I", str(here),
                    str(here / "test_main.cpp"), "-o", str(binary)], check=True)
    for scenario in scenarios:
        result = subprocess.run([str(binary), scenario], capture_output=True, text=True)
        if result.returncode:
            raise SystemExit(f"FAIL {scenario}\n{result.stdout}\n{result.stderr}")
        for line in result.stdout.splitlines():
            if line.startswith("{"):
                payload = json.loads(line)
                assert payload["simulated"] is True
                assert payload["delivered"] is False
                assert payload["transport"] == "serial_placeholder"
        print(f"PASS {scenario}")
print(f"All {len(scenarios)} host scenarios passed (no physical hardware tested).")
