#!/usr/bin/env python3
"""patch_px4.py - edit YOUR px4_run_scenario.py in place (keeps your campus layout).

  python3 patch_px4.py px4_run_scenario.py     # makes px4_run_scenario.py.bak first

1. replaces the failure scenarios PX4 rejects (baro/accel/mag 'wrong', gps 'intermittent') with supported ones
2. catches PX4 'UNSUPPORTED' errors, prints and records them instead of crashing
3. sets CA_FAILURE_MODE=1 (needed for motor-failure injection)
Safe to run twice.
"""
import re
import shutil
import sys

path = sys.argv[1] if len(sys.argv) > 1 else "px4_run_scenario.py"
s = open(path).read()
if "PX4 REJECTED" in s:
    sys.exit("already patched")
shutil.copy(path, path + ".bak")

new = '''    # PX4 accepts only some failure types per unit: baro/accel/gyro/mag = OFF or STUCK; GPS = OFF/STUCK/WRONG
    # (WRONG only changes the fix type); motor and battery = OFF or WRONG. Others return FailureError UNSUPPORTED.
    "baro_stuck": [{"t": 15, "do": "inject", "unit": "SENSOR_BARO", "type": "STUCK"}],
    "baro_off": [{"t": 15, "do": "inject", "unit": "SENSOR_BARO", "type": "OFF"}],
    "accel_stuck": [{"t": 20, "do": "inject", "unit": "SENSOR_ACCEL", "type": "STUCK"}],
    "accel_off": [{"t": 20, "do": "inject", "unit": "SENSOR_ACCEL", "type": "OFF"}],
    "gyro_stuck": [{"t": 20, "do": "inject", "unit": "SENSOR_GYRO", "type": "STUCK"}],
    "gyro_off": [{"t": 20, "do": "inject", "unit": "SENSOR_GYRO", "type": "OFF"}],
    "mag_stuck": [{"t": 20, "do": "inject", "unit": "SENSOR_MAG", "type": "STUCK"}],
    "mag_off": [{"t": 20, "do": "inject", "unit": "SENSOR_MAG", "type": "OFF"}],
    "motor_off": [{"t": 20, "do": "inject", "unit": "SYSTEM_MOTOR", "type": "OFF", "instance": 1}],
    "motor_wrong": [{"t": 20, "do": "inject", "unit": "SYSTEM_MOTOR", "type": "WRONG", "instance": 1}],
    "battery_off": [{"t": 20, "do": "inject", "unit": "SYSTEM_BATTERY", "type": "OFF"}],
    "battery_wrong": [{"t": 20, "do": "inject", "unit": "SYSTEM_BATTERY", "type": "WRONG"}],
'''
# drop old unsupported / replaced lines (single-line entries), then insert the new block before the spoof scenarios
for name in ["baro_stuck", "baro_wrong", "accel_wrong", "gyro_stuck", "mag_wrong", "motor_off", "gps_intermittent"]:
    s = re.sub(r'^[ \t]*"%s": \[.*\],?\n' % name, "", s, flags=re.M)
i = s.index('    "gps_spoof_freeze"')
s = s[:i] + new + s[i:]

old = '                    await drone.failure.inject(FAILURE_UNITS[a["unit"]], FAILURE_TYPES[a["type"]], a.get("instance", 0))\n'
assert old in s, "inject line not found - send me your px4_run_scenario.py"
s = s.replace(old, '''                    try:
                        await drone.failure.inject(FAILURE_UNITS[a["unit"]], FAILURE_TYPES[a["type"]], a.get("instance", 0))
                    except Exception as ex:      # PX4 refuses unsupported unit/type pairs: record it, keep flying
                        print(f"[d{idx}] PX4 REJECTED this failure (not supported): {ex}")
                        a["error"] = "rejected by PX4"
''')
p = '    await drone.param.set_param_int("SYS_FAILURE_EN", 1)\n'
assert p in s, "SYS_FAILURE_EN line not found"
s = s.replace(p, p + '''    try:
        await drone.param.set_param_int("CA_FAILURE_MODE", 1)       # allow motor-failure injection
    except Exception as ex:
        print(f"[d{idx}] CA_FAILURE_MODE not set ({ex})")
''', 1)
open(path, "w").write(s)
print("patched", path, "(backup:", path + ".bak)")
