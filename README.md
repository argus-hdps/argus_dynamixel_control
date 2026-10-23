# SBIG Camera DYNAMIXEL API

A small Python API for controlling the DYNAMIXEL servo port on the SBIG camera control board over its fiber-optic network connection.

The project was reverse-engineered against one connected DYNAMIXEL XL431-T250-T (model 1061). It sends standard DYNAMIXEL Protocol 2.0 packets over UDP to the camera; the camera forwards them to the servo bus.

## Current verified setup

| Setting | Value |
| --- | --- |
| Camera | SBIG AC461M, serial AC461M-007313-13 |
| Camera bridge IP | 10.0.144.35 |
| Argus fiber host IP | 10.0.144.2 |
| UDP port | 6464 |
| Protocol | DYNAMIXEL Protocol 2.0 |
| Servo model | XL431-T250-T, model number 1061 |
| Current servo ID | 102 |
| Operating mode | Position mode (3) |
| Position limits | 0..16383 counts |
| Last confirmed state | Torque off, no hardware error; position 8192 |

The API validates model, mode, limits, position, goal, torque, temperature, and hardware error before movement. Once validated on a Servo object, movement calls refresh torque and hardware-error state before each move while reusing the validated model and limits. The example defaults to ID 102, but the ID can be changed on the command line.

## Quick start

The package uses only the Python standard library.

    cd camera-servo-api
    python3 -m pip install -e .

Read the servo state:

    python3 - <<'PY'
    from camera_dynamixel import CameraBridge

    bridge = CameraBridge()
    servo = bridge.servo(102)
    print(servo.status().as_dict())
    PY

Move to a centered angle:

    python3 - <<'PY'
    from camera_dynamixel import CameraBridge

    servo = CameraBridge().servo(102)
    was_enabled = servo.torque_enabled
    try:
        if not was_enabled:
            servo.enable_torque()
        servo.move_to_angle(90)
        print("position:", servo.position)
    finally:
        servo.move_to_angle(0)
        if not was_enabled:
            servo.disable_torque()
    PY

The API does not change EEPROM configuration. Goal-position writes are read back because the camera bridge may drop write acknowledgements.

## Angle-cycle demo

This standalone demo is separate from the MIDI experiment. It cycles through:

    0 -> 90 -> 180 -> -90 -> -180 -> 0 degrees

Run two cycles by default from the repository root:

    python3 examples/angle_cycle.py

The demo waits for each commanded position, but adds no extra pause between positions by default. Use `--dwell 0.5` to add a half-second pause. The observed profile-velocity and profile-acceleration registers are both 0, so this demo does not rewrite the servo speed settings.

The example also works directly from its own directory:

    cd examples
    python3 angle_cycle.py

Choose an ID and cycle count:

    python3 examples/angle_cycle.py --id 102 --cycles 3 --retries 10 --timeout 0.45

The angle convention is centered on the servo's reported range: -180 degrees maps to the minimum position, 0 degrees maps to the midpoint, and +180 degrees maps to the maximum. For model 1061 with limits 0..16383, the targets are approximately 0, 4096, 8192, 12288, and 16383 counts. This is an encoder-range mapping for a free-standing demonstration; it is not calibrated to telescope focus or a mechanical zero.

The demo checks the model, mode, hardware error, limits, and the goal-to-present gap before enabling torque. If torque was initially off, it returns to 0 degrees and turns torque off after the cycle. It prints each requested angle, goal count, and present count. Arrival tolerance defaults to 32 encoder counts (about 0.70 degrees under this mapping), the move timeout defaults to 30 seconds, and the transport retries dropped replies. A complete one-cycle hardware run on 2026-10-23 reached counts 8192, 12278, 16353, 4120, 3, and 8185 for 0, +90, +180, -90, -180, and 0 degrees. Cleanup returned to 8184 and restored torque off; a following status read confirmed position and goal 8192, torque off, and hardware error 0. The largest target error was 30 counts at +180. Stop the process with Ctrl-C; cleanup also attempts to return to 0 degrees and restore the original torque state.

## API overview

    from camera_dynamixel import CameraBridge

    bridge = CameraBridge(
        camera_ip="10.0.144.35",
        host_ip="10.0.144.2",
        udp_port=6464,
    )
    servo = bridge.servo(102)

    state = servo.status()
    servo.enable_torque()
    servo.move_to_angle(45)
    servo.move_to_position(8192)
    servo.disable_torque()

Main methods and properties:

| API | Description |
| --- | --- |
| servo.status() | Read all documented status fields |
| servo.position | Read present encoder position |
| servo.goal_position | Read goal encoder position |
| servo.angle | Read position as centered degrees |
| servo.enable_torque() | Enable motor torque |
| servo.disable_torque() | Disable motor torque |
| servo.move_to_angle(degrees) | Move within -180..+180 degrees |
| servo.move_to_position(counts) | Move to an absolute encoder count |
| servo.read_register(address, size) | Read a raw control-table value |

Moves require torque to be enabled. The convenience methods reject unsupported models, non-position mode, hardware errors, and targets outside the live limits.

## Transport details

The vendor-modified ROBOTIS SDK and email confirmed the bridge protocol. This repository implements Protocol 2.0 packet framing and CRC directly, so the private vendor SDK is not a runtime dependency.

- Send an ordinary DYNAMIXEL Protocol 2.0 instruction packet as the UDP payload to camera port 6464.
- Bind the host side to 10.0.144.2 and use an ephemeral source port.
- Replies contain a four-byte big-endian length prefix followed by a normal DYNAMIXEL status packet.
- Baud rate is not used by the UDP bridge.
- The API verifies packet CRC and response ID.
- Reads retry; writes are verified by register readback.

## Control-table addresses used

| Address | Size | Field |
| ---: | ---: | --- |
| 0 | 2 | Model number |
| 6 | 1 | Firmware version |
| 7 | 1 | ID |
| 11 | 1 | Operating mode |
| 48 | 4 | Maximum position |
| 52 | 4 | Minimum position |
| 64 | 1 | Torque enable |
| 70 | 1 | Hardware error |
| 116 | 4 | Goal position |
| 132 | 4 | Present position |
| 146 | 1 | Present temperature |

These addresses and values were read from the connected model 1061. The API intentionally targets this servo model and does not support the different model 1060 units that were briefly connected during testing.

## MIDI experiment

The earlier TNG MIDI experiment is separate from this angle-cycle example and remains outside this repository. It produced twitchy motion rather than a discernible melody, so this repository does not present it as a successful music player.

## Repository layout

    src/camera_dynamixel/     Protocol 2.0 transport and servo API
    examples/angle_cycle.py  Independent angle-cycle hardware demo
    docs/protocol.md          Reverse-engineering notes
    pyproject.toml            Editable-install metadata

Private vendor email, firmware archives, and MIDI files are kept outside this repository and excluded from Git.
