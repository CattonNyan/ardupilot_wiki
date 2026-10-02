"""Compile documentation excerpts in a disposable ArduPilot checkout."""
import argparse
from pathlib import Path
import re
import subprocess
import textwrap
import time


def literal_blocks(source):
    lines = source.splitlines()
    blocks = []
    for index, line in enumerate(lines):
        if line.strip() != "::":
            continue
        start = index + 1
        while start < len(lines) and not lines[start].strip():
            start += 1
        indent = len(lines[start]) - len(lines[start].lstrip())
        end = start
        while end < len(lines):
            current = lines[end]
            if current.strip() and len(current) - len(current.lstrip()) < indent:
                break
            end += 1
        blocks.append(textwrap.dedent("\n".join(lines[start:end])).rstrip())
    return blocks


def replace_function(source, signature, replacement):
    start = source.index(signature)
    opening = source.index("{", start)
    depth = 1
    end = opening + 1
    while depth:
        depth += (source[end] == "{") - (source[end] == "}")
        end += 1
    return source[:start] + replacement + source[end:]


def prepare(firmware, guide):
    blocks = literal_blocks(guide.read_text(encoding="utf-8"))
    enum, declaration, capabilities, rtl, run, members, mapping, _ = blocks
    mode_header = firmware / "ArduCopter/mode.h"
    header = mode_header.read_text()
    enum_match = re.search(r"enum class Number : uint8_t\s*\{(.*?)\};", header, re.S)
    assert enum_match
    assert not re.search(r"=\s*100\s*[,}]", enum_match[1]), "Example number is already used"
    assert "NEW_MODE =    100" in enum
    header = header[:enum_match.end() - 2] + "    NEW_MODE = 100,\n    " + header[enum_match.end() - 2:]
    header += "\nclass ModeNewMode : public Mode {\n" + declaration
    header += "\npublic:\n" + capabilities + "\n};\n"
    mode_header.write_text(header)

    copter_header = firmware / "ArduCopter/Copter.h"
    source = copter_header.read_text()
    assert "ModeNewMode mode_newmode;" in members
    assert source.count("ModeStabilize mode_stabilize;") == 1
    copter_header.write_text(source.replace("ModeStabilize mode_stabilize;",
                                          "ModeStabilize mode_stabilize;\n    ModeNewMode mode_newmode;"))

    mode_cpp = firmware / "ArduCopter/mode.cpp"
    source = mode_cpp.read_text()
    start = source.index("Mode *Copter::mode_from_mode_num(")
    switch = source.index("switch (mode) {", start)
    case = re.search(r"case Mode::Number::NEW_MODE:.*?break;", mapping, re.S)[0]
    # Compile the mapping example as written, with its local result variable.
    addition = "\n        case Mode::Number::NEW_MODE: {\n            Mode *ret = nullptr;\n"
    addition += case[case.index(":", case.index("NEW_MODE")) + 1:]
    addition = addition.replace("break;", "return ret;") + "\n        }\n"
    at = switch + len("switch (mode) {")
    mode_cpp.write_text(source[:at] + addition + source[at:])

    (firmware / "ArduCopter/mode_newmode.cpp").write_text(
        '#include "Copter.h"\n'
        'bool ModeNewMode::init(bool) { return true; }\n' +
        run.replace("ModeStabilize::run()", "ModeNewMode::run()") + "\n")
    rtl_file = firmware / "ArduCopter/mode_rtl.cpp"
    rtl_file.write_text(replace_function(rtl_file.read_text(), "bool ModeRTL::init(bool ignore_checks)", rtl))
    print("Extracted and inserted the class, capability, run, RTL init and registration examples.")
    print("Disposable test checkout only; no firmware changes are submitted upstream.")


def smoke(firmware):
    from pymavlink import mavutil
    firmware = firmware.resolve()
    command = [str(firmware / "build/sitl/bin/arducopter"), "-S", "--model", "quad",
               "--speedup", "1", "--home", "-35.362938,149.165085,585,0",
               "--defaults", str(firmware / "Tools/autotest/default_params/copter.parm")]
    with open("sitl-smoke.log", "w") as log:
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT)
        connection = None
        try:
            for _ in range(40):
                if process.poll() is not None:
                    raise RuntimeError("SITL exited during startup")
                try:
                    connection = mavutil.mavlink_connection("tcp:127.0.0.1:5760", source_system=255)
                    break
                except OSError:
                    time.sleep(1)
            assert connection is not None, "SITL TCP port did not become ready"
            assert connection.wait_heartbeat(timeout=30), "No heartbeat"
            heartbeat = connection.recv_match(type="HEARTBEAT", blocking=True, timeout=5)
            assert heartbeat is not None
            assert not heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
            connection.mav.command_long_send(
                connection.target_system, connection.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 100, 0, 0, 0, 0, 0)
            deadline = time.monotonic() + 20
            while time.monotonic() < deadline:
                heartbeat = connection.recv_match(type="HEARTBEAT", blocking=True, timeout=2)
                if heartbeat is not None and heartbeat.custom_mode == 100:
                    assert not heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                    print("PASS: entered example mode 100 while disarmed; no takeoff or flight test performed.")
                    return
            raise RuntimeError("Did not enter example mode 100")
        finally:
            if connection is not None:
                connection.close()
            process.terminate()
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("action", choices=["prepare", "smoke"])
    parser.add_argument("firmware", type=Path)
    parser.add_argument("--guide", type=Path)
    args = parser.parse_args()
    if args.action == "prepare":
        prepare(args.firmware, args.guide)
    else:
        smoke(args.firmware)
