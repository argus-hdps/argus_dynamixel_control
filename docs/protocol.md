# Reverse-engineering notes

## Confirmed path

The SBIG AC461M camera control board accepts DYNAMIXEL Protocol 2.0 instructions over UDP port 6464 on its primary IPv4 address. On Argus, the fiber NIC is 10.0.144.2 and the camera is 10.0.144.35. The command payload is the normal DYNAMIXEL instruction packet. Replies are prefixed with a four-byte big-endian packet length.

The baud-rate option in the vendor SDK is compatibility-only for this UDP bridge. No raw sockets, elevated privileges, firewall changes, or camera firmware changes are needed.

## Confirmed servo

The current supported unit is ID 102, model number 1061, XL431-T250-T. It has reported firmware 48, position mode 3, position limits 0..16383, and a 14-bit position range. Live reads have reported temperatures around 22-25 C and hardware error 0. The control-table addresses in the README are live-verified for this unit.

A prior single-servo bench move changed the encoder position by 16 counts and returned close to its starting point. The later MIDI experiments were not a successful performance. A complete angle-cycle demo was telemetry-confirmed on 2026-10-23 with torque initially off. Requested angles 0, +90, +180, -90, -180, 0 reached counts 8192, 12278, 16353, 4120, 3, 8185; cleanup returned to count 8184 and torque off. The maximum observed target error was 30 counts (about 0.66 degrees using the software mapping). The latest status read after the run showed position and goal 8192, torque off, hardware error 0, temperature 26 C.

## Angle convention in this package

The package maps the full reported position range to one 360-degree revolution and uses its midpoint as 0 degrees. Thus 0 degrees is near count 8192; -180 is count 0; +180 is count 16383. This is an explicit software convention for the free-servo demo, not a telescope focus calibration. The model-specific physical degree-per-count value has not been independently measured with a protractor.

## Packet format

Instruction frame:

    FF FF FD 00 | ID | LENGTH_L LENGTH_H | INSTRUCTION | PARAMETERS | CRC_L CRC_H

The four-byte camera reply prefix is not included in the CRC. The DYNAMIXEL status packet begins with the same header, has instruction/status marker 0x55, and includes an error byte followed by any read data.

## Observed bridge behavior

The bridge sometimes drops replies, especially after address scans or when multiple devices with different control tables were attached. Keep transactions paced and verify writes by reading the register back. Status replies were intermittently dropped during demo attempts; the successful full-cycle run used 10 retries per transaction and 32-count arrival tolerance. The single supported model is 1061; the temporary model 1060 units are intentionally not supported by this package.
