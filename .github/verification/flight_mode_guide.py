"""Apply documented excerpts without rewriting their control flow, then test SITL."""
import argparse
import json
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
        indent = len(line) - len(line.lstrip()) + 1
        end = start
        while end < len(lines):
            current = lines[end]
            if current.strip() and len(current) - len(current.lstrip()) < indent:
                break
            end += 1
        blocks.append(textwrap.dedent("\n".join(lines[start:end])).rstrip())
    return blocks


def brace_end(source, opening):
    depth = 1
    end = opening + 1
    while depth:
        depth += (source[end] == "{") - (source[end] == "}")
        end += 1
    return end


def function_bounds(source, signature):
    start = source.index(signature)
    return start, brace_end(source, source.index("{", start))


def replace_function(source, signature, replacement):
    start, end = function_bounds(source, signature)
    return source[:start] + replacement + source[end:]


def tokens(source):
    return re.sub(r"\s+", "", source)


def append_array(source, signature, entry):
    start, end = function_bounds(source, signature)
    function = source[start:end]
    opening = function.index("{", function.index("modes[]"))
    at = brace_end(function, opening) - 1
    function = function[:at] + "    " + entry + "\n    " + function[at:]
    return source[:start] + function + source[end:]


def prepare(firmware, guide):
    blocks = literal_blocks(guide.read_text(encoding="utf-8"))
    enum, declaration, capabilities, rtl, run, members, mapping, available, blocking, parameters = blocks
    mode_header = firmware / "ArduCopter/mode.h"
    header = mode_header.read_text()
    enum_match = re.search(r"enum class Number : uint8_t\s*\{(.*?)\};", header, re.S)
    assert enum_match
    entries = dict(re.findall(r"(\w+)\s*=\s*(\d+)\s*,", enum_match[1]))
    documented = dict(re.findall(r"(\w+)\s*=\s*(\d+)\s*,", enum))
    number = int(documented.pop("NEW_MODE"))
    assert entries == documented, "Documented mode enum differs from upstream"
    reserved = re.findall(r"Mode number (\d+) reserved", enum_match[1])
    assert number not in map(int, entries.values())
    assert number not in map(int, reserved)
    assert set(reserved) == set(re.findall(r"Mode number (\d+) reserved", enum))
    assert number == 99
    flip_script = (firmware / "libraries/AP_Scripting/examples/Flip_Mode.lua").read_text()
    assert re.search(r"local MODE_NUMBER = 100\b", flip_script)
    assert number != 100, "Native mode must not collide with the official Lua example"
    # Install the complete documented enum, including reservations.
    header = header[:enum_match.start()] + enum[enum.index("enum class"):].rstrip() + header[enum_match.end():]
    header += "\nclass ModeNewMode : public Mode {\n" + declaration
    header += "\npublic:\n" + capabilities + "\n};\n"
    mode_header.write_text(header)

    copter_header = firmware / "ArduCopter/Copter.h"
    source = copter_header.read_text()
    member_match = re.search(
        r"#if MODE_ACRO_ENABLED\s*\n#if FRAME_CONFIG == HELI_FRAME.*?ModeAutoTune mode_autotune;\s*\n#endif",
        source, re.S)
    assert member_match
    assert tokens(members.replace("ModeNewMode mode_newmode;", "")) == tokens(member_match[0])
    # Install the entire member excerpt, including all preprocessor guards.
    copter_header.write_text(source[:member_match.start()] + members + source[member_match.end():])

    mode_cpp = firmware / "ArduCopter/mode.cpp"
    source = mode_cpp.read_text()
    signature = "Mode *Copter::mode_from_mode_num("
    start, end = function_bounds(source, signature)
    original = source[start:end]
    omitted_start = original.index("return &mode_stabilize;") + len("return &mode_stabilize;")
    omitted_end = original.index("        default:", omitted_start)
    switch_end = brace_end(original, original.index("{", original.index("switch (mode)")))
    lua_end = original.rindex("    return nullptr;")
    # Fill only the two omissions explicitly marked in the guide. No return/break rewriting.
    expanded = mapping.replace("// Other existing mode cases omitted from this excerpt.",
                               original[omitted_start:omitted_end].strip())
    expanded = expanded.replace("// Existing Lua mode lookup omitted from this excerpt.",
                                original[switch_end:lua_end].strip())
    expanded = expanded[expanded.index("Mode *Copter::mode_from_mode_num("):]
    new_case = re.search(r"case Mode::Number::NEW_MODE:\s*return &mode_newmode;", expanded)
    assert new_case
    assert tokens(expanded[:new_case.start()] + expanded[new_case.end():]) == tokens(original)
    source = source[:start] + expanded + source[end:]

    assert available == "&copter.mode_newmode,"
    source = append_array(source, "uint32_t Copter::get_available_mode_enabled_mask()", available)
    gcs_file = firmware / "ArduCopter/GCS_MAVLink_Copter.cpp"
    gcs_file.write_text(append_array(gcs_file.read_text(),
                                   "uint8_t GCS_MAVLINK_Copter::send_available_mode(", available))

    assert blocking == "(uint8_t)Mode::Number::TURTLE,\n(uint8_t)Mode::Number::NEW_MODE,"
    start, end = function_bounds(source, "bool Copter::gcs_mode_enabled(")
    function = source[start:end]
    opening = function.index("{", function.index("mode_list []"))
    closing = brace_end(function, opening) - 1
    bit = len(re.findall(r"\(uint8_t\)Mode::Number::", function[opening:closing]))
    assert bit < 32
    # Install the documented ending verbatim: do not silently repair a missing comma.
    last = re.search(r"\(uint8_t\)Mode::Number::TURTLE\s*$", function[:closing])
    assert last, "Upstream array ending changed"
    function = function[:last.start()] + textwrap.indent(blocking, "        ").lstrip() + "\n    " + function[closing:]
    mode_cpp.write_text(source[:start] + function + source[end:])

    vehicle_file = firmware / "libraries/AP_Vehicle/AP_Vehicle.cpp"
    vehicle = vehicle_file.read_text()
    bitmask = re.search(r"(    // @Bitmask\{Copter\}: \d+:[^\n]+\n)(?=    // @Bitmask\{Plane\})", vehicle)
    assert bitmask and int(re.search(r": (\d+):", bitmask[1])[1]) == bit - 1
    vehicle = vehicle[:bitmask.end()] + f"    // @Bitmask{{Copter}}: {bit}:NewMode\n" + vehicle[bitmask.end():]
    vehicle_file.write_text(vehicle)

    parameter_file = firmware / "ArduCopter/Parameters.cpp"
    source = parameter_file.read_text()
    start = source.index("    // @Param: FLTMODE1\n")
    end = source.index("    // @Param: FLTMODE3\n", start)
    existing = source[start:end]
    assert tokens(parameters.replace(",99:NewMode", "")) == tokens(existing)
    parameter_file.write_text(source[:start] + textwrap.indent(parameters, "    ") + "\n\n" + source[end:])

    # The run excerpt is deliberately shortened, and init is a test-only entry stub.
    (firmware / "ArduCopter/mode_newmode.cpp").write_text(
        '#include "Copter.h"\n'
        'bool ModeNewMode::init(bool) { return true; }\n' +
        run.replace("ModeStabilize::run()", "ModeNewMode::run()") + "\n")
    rtl_file = firmware / "ArduCopter/mode_rtl.cpp"
    rtl_file.write_text(replace_function(rtl_file.read_text(), "bool ModeRTL::init(bool ignore_checks)", rtl))
    Path("example-metadata.json").write_text(json.dumps({"mode_number": number, "gcs_block_bit": bit}, indent=2))
    print(f"Applied complete enum/member/parameter excerpts and direct-return lookup; GCS-block bit {bit}.")
    print("Only explicitly marked existing-case/Lua omissions filled; run class renamed; init stub added.")
    print("RTL excerpt applied with its documented omissions. Compile check only; no RTL flight test.")


def smoke(firmware):
    import os
    os.environ["MAVLINK20"] = "1"
    from pymavlink import mavutil
    from pymavlink.generator.mavgen import Opts, mavgen

    firmware = firmware.resolve()
    # AVAILABLE_MODES is in development.xml at this firmware revision, not ardupilotmega.xml.
    dialect = Path(__file__).resolve().with_name("flight_mode_dialect")
    assert mavgen(Opts(str(dialect), wire_protocol="2.0", language="Python", validate=False),
                  [str(firmware / "modules/mavlink/message_definitions/v1.0/development.xml")])
    import flight_mode_dialect
    mavutil.mavlink = flight_mode_dialect
    metadata = json.loads(Path("example-metadata.json").read_text())
    number = metadata["mode_number"]
    block_mask = 1 << metadata["gcs_block_bit"]
    extra_defaults = Path("sitl-verification.parm").resolve()
    extra_defaults.write_text("SERIAL0_PROTOCOL 2\nSCR_ENABLE 1\nSCR_HEAP_SIZE 262144\n")
    scripts = Path("scripts")
    scripts.mkdir(exist_ok=True)
    # Run the actual upstream example unchanged alongside native mode 99.
    (scripts / "Flip_Mode.lua").write_bytes(
        (firmware / "libraries/AP_Scripting/examples/Flip_Mode.lua").read_bytes())
    (scripts / "verify_native_collision.lua").write_text(
        f'local function update()\n'
        f'  assert(vehicle:register_custom_mode({number}, "Collision", "CLSN") == nil)\n'
        f'  gcs:send_text(6, "PASS: native mode {number} rejects Lua reuse")\n'
        f'  return update, 1000\nend\nreturn update()\n')
    command = [str(firmware / "build/sitl/bin/arducopter"), "--model", "quad",
               "--speedup", "1", "--home", "-35.362938,149.165085,585,0",
               "--defaults", f"{firmware / 'Tools/autotest/default_params/copter.parm'},{extra_defaults}"]
    with open("sitl-smoke.log", "w") as log:
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT)
        connection = None
        try:
            for _ in range(40):
                if process.poll() is not None:
                    raise RuntimeError("SITL exited during startup")
                try:
                    connection = mavutil.mavlink_connection("tcp:127.0.0.1:5760", source_system=255)
                    # Keep the generated parser even if the first heartbeat uses MAVLink 1 framing.
                    connection.first_byte = False
                    connection.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                                                  mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                                                  0, 0, mavutil.mavlink.MAV_STATE_ACTIVE)
                    break
                except OSError:
                    time.sleep(1)
            assert connection is not None, "SITL TCP port did not become ready"
            assert connection.wait_heartbeat(timeout=30), "No heartbeat"
            assert connection.mav.__class__.__module__ == flight_mode_dialect.__name__, connection.mav.__class__
            print(f"Parser: {connection.mav.__class__.__module__}; wire protocol {connection.WIRE_PROTOCOL_VERSION}")

            def drain():
                while connection.recv_match(blocking=False) is not None:
                    pass

            def send_command(command_id, *params):
                drain()
                connection.mav.command_long_send(
                    connection.target_system, connection.target_component,
                    command_id, 0, *(list(params) + [0] * (7 - len(params))))

            def wait_for(message_type, predicate, timeout=15):
                deadline = time.monotonic() + timeout
                while time.monotonic() < deadline:
                    message = connection.recv_match(blocking=True, timeout=1)
                    if message is not None and message.get_type() in ("COMMAND_ACK", "AVAILABLE_MODES", "BAD_DATA"):
                        print(f"RECV: {message}")
                    if message is not None and message.get_type() == message_type and predicate(message):
                        return message
                raise RuntimeError(f"Timed out waiting for {message_type}")

            def set_block(value):
                drain()
                connection.mav.param_set_send(
                    connection.target_system, connection.target_component,
                    b"FLTMODE_GCSBLOCK", value, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
                wait_for("PARAM_VALUE", lambda m: m.param_id == "FLTMODE_GCSBLOCK" and int(m.param_value) == value)

            def mode_command(value, expected):
                send_command(mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                             mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, value)
                ack = wait_for("COMMAND_ACK", lambda m: m.command == mavutil.mavlink.MAV_CMD_DO_SET_MODE)
                assert ack.result == expected, ack

            def heartbeat(value):
                hb = wait_for("HEARTBEAT", lambda m: m.custom_mode == value)
                assert not hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED, hb
                assert hb.get_msgbuf()[0] == 253, "Expected MAVLink 2 framing"

            def available(blocked):
                send_command(mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
                             mavutil.mavlink.MAVLINK_MSG_ID_AVAILABLE_MODES, 1)
                first = wait_for("AVAILABLE_MODES", lambda m: m.mode_index == 1)
                found = []
                lua_modes = []
                for index in range(2, first.number_modes + 1):
                    send_command(mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
                                 mavutil.mavlink.MAVLINK_MSG_ID_AVAILABLE_MODES, index)
                    item = wait_for("AVAILABLE_MODES", lambda m: m.mode_index == index)
                    if item.custom_mode == number:
                        found.append(item)
                    if item.custom_mode == 100:
                        lua_modes.append(item)
                assert len(found) == 1, f"Expected mode {number} once in {first.number_modes} available modes"
                message = found[0]
                assert message.mode_name == "NEWMODE", message
                flag = mavutil.mavlink.MAV_MODE_PROPERTY_NOT_USER_SELECTABLE
                assert bool(message.properties & flag) == blocked, message
                assert len(lua_modes) == 1 and lua_modes[0].mode_name == "Flip 2", lua_modes
                print(f"PASS: AVAILABLE_MODES mode {number}, name {message.mode_name}, blocked={blocked}")
                print("PASS: unmodified Flip_Mode.lua registered mode 100 alongside native mode 99")

            ready = wait_for("HEARTBEAT", lambda m: m.system_status == mavutil.mavlink.MAV_STATE_STANDBY, timeout=60)
            assert not ready.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
            print(f"SITL initialized: status {ready.system_status}, framing {ready.get_msgbuf()[0]}")
            collision = wait_for("STATUSTEXT", lambda m: m.text == f"PASS: native mode {number} rejects Lua reuse", timeout=30)
            print(collision.text)
            set_block(0)
            heartbeat(0)
            available(False)
            mode_command(number, mavutil.mavlink.MAV_RESULT_ACCEPTED)
            heartbeat(number)
            print(f"PASS: entered mode {number} while disarmed")
            mode_command(0, mavutil.mavlink.MAV_RESULT_ACCEPTED)
            heartbeat(0)
            send_command(mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                         mavutil.mavlink.MAVLINK_MSG_ID_AVAILABLE_MODES_MONITOR, 1000000)
            baseline = wait_for("AVAILABLE_MODES_MONITOR", lambda m: True)
            set_block(block_mask)
            wait_for("AVAILABLE_MODES_MONITOR", lambda m: m.seq != baseline.seq)
            print("PASS: AVAILABLE_MODES_MONITOR sequence changed after GCS blocking")
            available(True)
            mode_command(number, mavutil.mavlink.MAV_RESULT_FAILED)
            heartbeat(0)
            print("PASS: blocked GCS mode command failed and vehicle stayed in Stabilize")
            set_block(0)
            available(False)
            mode_command(number, mavutil.mavlink.MAV_RESULT_ACCEPTED)
            heartbeat(number)
            mode_command(0, mavutil.mavlink.MAV_RESULT_ACCEPTED)
            heartbeat(0)
            print("PASS: clearing GCS block restored mode selection; no arming, takeoff or flight tests")
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
