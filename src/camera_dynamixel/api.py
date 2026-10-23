"""Small Protocol 2.0 API for the SBIG camera's DYNAMIXEL UDP bridge."""

from __future__ import annotations

from dataclasses import asdict, dataclass, replace
import socket
import struct
import time
from typing import Optional

DEFAULT_CAMERA_IP = "10.0.144.35"
DEFAULT_HOST_IP = "10.0.144.2"
DEFAULT_UDP_PORT = 6464
SUPPORTED_MODELS = {1061}

# XL431-T250-T control table entries observed on the Argus camera.
REG_MODEL = (0, 2)
REG_FIRMWARE = (6, 1)
REG_ID = (7, 1)
REG_OPERATING_MODE = (11, 1)
REG_MAX_POSITION = (48, 4)
REG_MIN_POSITION = (52, 4)
REG_TORQUE_ENABLE = (64, 1)
REG_HARDWARE_ERROR = (70, 1)
REG_GOAL_POSITION = (116, 4)
REG_PRESENT_POSITION = (132, 4)
REG_TEMPERATURE = (146, 1)


class ServoError(RuntimeError):
    """Raised when the servo reports an error or a requested action is invalid."""


class ServoTimeout(TimeoutError):
    """Raised when the camera bridge does not return a valid status packet."""


@dataclass(frozen=True)
class ServoStatus:
    model_number: int
    firmware_version: int
    servo_id: int
    operating_mode: int
    torque_enabled: bool
    hardware_error: int
    minimum_position: int
    maximum_position: int
    goal_position: int
    present_position: int
    temperature_c: int

    def as_dict(self) -> dict:
        return asdict(self)


def _crc16(data: bytes) -> int:
    """DYNAMIXEL Protocol 2.0 CRC-16 (polynomial 0x8005)."""
    crc = 0
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x8005) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def _instruction_packet(servo_id: int, instruction: int, parameters: bytes = b"") -> bytes:
    if not 0 <= servo_id <= 254:
        raise ValueError("DYNAMIXEL ID must be in the range 0..254")
    body = bytearray(
        b"\xff\xff\xfd\x00"
        + bytes((servo_id,))
        + struct.pack("<H", len(parameters) + 3)
        + bytes((instruction,))
        + parameters
    )
    body.extend(struct.pack("<H", _crc16(body)))
    return bytes(body)


def _status_from_datagram(datagram: bytes) -> Optional[bytes]:
    """Strip camera framing and return one complete DYNAMIXEL status packet."""
    if len(datagram) < 4:
        return None
    size = struct.unpack(">I", datagram[:4])[0]
    if size < 11 or size > len(datagram) - 4:
        return None
    frame = datagram[4 : 4 + size]
    if frame[:4] != b"\xff\xff\xfd\x00":
        return None
    if _crc16(frame[:-2]) != struct.unpack("<H", frame[-2:])[0]:
        return None
    if frame[7] != 0x55:
        return None
    return frame


class CameraBridge:
    """UDP transport for the camera's Protocol 2.0 DYNAMIXEL bridge.

    The bridge carries ordinary DYNAMIXEL packets as UDP payloads. Replies have
    a four-byte big-endian length prefix. Each request uses a fresh ephemeral
    source port on the host fiber address.
    """

    def __init__(
        self,
        camera_ip: str = DEFAULT_CAMERA_IP,
        host_ip: str = DEFAULT_HOST_IP,
        udp_port: int = DEFAULT_UDP_PORT,
        timeout: float = 0.45,
        retries: int = 10,
    ) -> None:
        self.camera_ip = camera_ip
        self.host_ip = host_ip
        self.udp_port = udp_port
        self.timeout = timeout
        self.retries = retries

    def _exchange(self, servo_id: int, instruction: int, parameters: bytes = b"") -> tuple[int, bytes]:
        request = _instruction_packet(servo_id, instruction, parameters)
        last_error = None
        for _ in range(self.retries):
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
                sock.settimeout(self.timeout)
                sock.bind((self.host_ip, 0))
                sock.sendto(request, (self.camera_ip, self.udp_port))
                deadline = time.monotonic() + self.timeout
                while time.monotonic() < deadline:
                    try:
                        datagram, peer = sock.recvfrom(2048)
                    except socket.timeout as exc:
                        last_error = exc
                        break
                    if peer != (self.camera_ip, self.udp_port):
                        continue
                    frame = _status_from_datagram(datagram)
                    if frame is None or frame[4] != servo_id:
                        continue
                    return frame[8], frame[9:-2]
        raise ServoTimeout(
            f"No valid response from camera bridge {self.camera_ip}:{self.udp_port} "
            f"for servo ID {servo_id}"
        ) from last_error

    def ping(self, servo_id: int) -> tuple[int, int]:
        """Return (model number, firmware version) for one servo ID."""
        _, data = self._exchange(servo_id, 0x01)
        if len(data) < 3:
            raise ServoError("Ping response did not contain model and firmware data")
        return int.from_bytes(data[:2], "little"), data[2]

    def read_register(self, servo_id: int, address: int, size: int) -> int:
        if size not in (1, 2, 4):
            raise ValueError("Register size must be 1, 2, or 4 bytes")
        parameters = struct.pack("<HH", address, size)
        last_error = None
        for attempt in range(2):
            try:
                error, data = self._exchange(servo_id, 0x02, parameters)
                if error:
                    raise ServoError(f"Servo {servo_id} returned status error 0x{error:02x}")
                if len(data) != size:
                    raise ServoError(f"Register {address} returned {len(data)} bytes; expected {size}")
                return int.from_bytes(data, "little")
            except ServoTimeout as exc:
                last_error = exc
                if attempt == 0:
                    time.sleep(0.1)
        raise last_error

    def _write_register(self, servo_id: int, address: int, value: int, size: int) -> None:
        """Write a register and verify it by readback.

        The camera may drop write acknowledgements, so readback is authoritative.
        """
        if size not in (1, 2, 4):
            raise ValueError("Register size must be 1, 2, or 4 bytes")
        if not 0 <= value < (1 << (size * 8)):
            raise ValueError(f"Value does not fit in {size} bytes")
        parameters = struct.pack("<H", address) + value.to_bytes(size, "little")
        last = None
        for attempt in range(3):
            try:
                self._exchange(servo_id, 0x03, parameters)
            except ServoTimeout:
                pass
            try:
                last = self.read_register(servo_id, address, size)
            except ServoTimeout:
                last = None
            if last == value:
                return
            time.sleep(0.05 * (attempt + 1))
        raise ServoError(
            f"Write verification failed at register {address}: "
            f"wanted {value}, last readback {last}"
        )

    def servo(self, servo_id: int) -> "Servo":
        return Servo(self, servo_id)


class Servo:
    """Traditional position/angle interface for an XL431-T250-T servo.

    Angles use a centered convention: 0 degrees is the midpoint of the
    reported position range, -180 is its minimum, and +180 is its maximum.
    """

    def __init__(self, bridge: CameraBridge, servo_id: int) -> None:
        if not 0 <= servo_id <= 252:
            raise ValueError("Servo ID must be in the range 0..252")
        self.bridge = bridge
        self.id = servo_id
        self._validated_status: Optional[ServoStatus] = None

    def read_register(self, address: int, size: int) -> int:
        return self.bridge.read_register(self.id, address, size)

    def _write_register(self, address: int, value: int, size: int) -> None:
        self.bridge._write_register(self.id, address, value, size)

    @property
    def position(self) -> int:
        return self.read_register(*REG_PRESENT_POSITION)

    @property
    def goal_position(self) -> int:
        return self.read_register(*REG_GOAL_POSITION)

    @property
    def torque_enabled(self) -> bool:
        return bool(self.read_register(*REG_TORQUE_ENABLE))

    def status(self) -> ServoStatus:
        model = self.read_register(*REG_MODEL)
        firmware = self.read_register(*REG_FIRMWARE)
        current_id = self.read_register(*REG_ID)
        mode = self.read_register(*REG_OPERATING_MODE)
        minimum = self.read_register(*REG_MIN_POSITION)
        maximum = self.read_register(*REG_MAX_POSITION)
        torque = bool(self.read_register(*REG_TORQUE_ENABLE))
        hardware_error = self.read_register(*REG_HARDWARE_ERROR)
        goal = self.read_register(*REG_GOAL_POSITION)
        position = self.read_register(*REG_PRESENT_POSITION)
        temperature = self.read_register(*REG_TEMPERATURE)
        return ServoStatus(
            model, firmware, current_id, mode, torque, hardware_error,
            minimum, maximum, goal, position, temperature
        )

    def validate(self, status: Optional[ServoStatus] = None) -> ServoStatus:
        status = status or self.status()
        if status.servo_id != self.id:
            raise ServoError(f"Expected servo ID {self.id}, read ID {status.servo_id}")
        if status.model_number not in SUPPORTED_MODELS:
            raise ServoError(
                f"Unsupported model {status.model_number}; this API targets model 1061"
            )
        if status.operating_mode != 3:
            raise ServoError(
                f"Servo {self.id} is in mode {status.operating_mode}; position mode 3 is required"
            )
        if status.hardware_error:
            raise ServoError(
                f"Servo {self.id} reports hardware error 0x{status.hardware_error:02x}"
            )
        if status.minimum_position >= status.maximum_position:
            raise ServoError("Servo reported invalid position limits")
        self._validated_status = status
        return status

    def _movement_status(self) -> ServoStatus:
        """Validate once, then refresh safety-critical live flags per move."""
        if self._validated_status is None:
            return self.validate()
        status = replace(
            self._validated_status,
            torque_enabled=bool(self.read_register(*REG_TORQUE_ENABLE)),
            hardware_error=self.read_register(*REG_HARDWARE_ERROR),
        )
        if status.hardware_error:
            raise ServoError(
                f"Servo {self.id} reports hardware error 0x{status.hardware_error:02x}"
            )
        return status

    @staticmethod
    def angle_to_position(angle_degrees: float, minimum: int, maximum: int) -> int:
        """Map -180..+180 degrees across the servo's reported position range."""
        if not -180.0 <= angle_degrees <= 180.0:
            raise ValueError("Angle must be between -180 and +180 degrees")
        count_span = maximum - minimum + 1
        raw = minimum + round((angle_degrees + 180.0) * count_span / 360.0)
        return min(maximum, max(minimum, raw))

    @staticmethod
    def position_to_angle(position: int, minimum: int, maximum: int) -> float:
        if not minimum <= position <= maximum:
            raise ValueError("Position is outside the servo-reported limits")
        count_span = maximum - minimum + 1
        return (position - minimum) * 360.0 / count_span - 180.0

    @property
    def angle(self) -> float:
        status = self.validate()
        return self.position_to_angle(status.present_position, status.minimum_position, status.maximum_position)

    def set_torque(self, enabled: bool) -> None:
        self._write_register(REG_TORQUE_ENABLE[0], int(bool(enabled)), REG_TORQUE_ENABLE[1])

    def enable_torque(self) -> None:
        self.set_torque(True)

    def disable_torque(self) -> None:
        self.set_torque(False)

    def _command_position(
        self,
        position: int,
        status: ServoStatus,
        *,
        wait: bool,
        timeout: float,
        tolerance: int,
    ) -> int:
        if not status.torque_enabled:
            raise ServoError("Torque is off; call enable_torque() before moving")
        if not status.minimum_position <= position <= status.maximum_position:
            raise ValueError(
                f"Position {position} outside {status.minimum_position}..{status.maximum_position}"
            )
        self._write_register(REG_GOAL_POSITION[0], position, REG_GOAL_POSITION[1])
        if not wait:
            return position
        deadline = time.monotonic() + timeout
        last_position = None
        while time.monotonic() < deadline:
            try:
                last_position = self.position
                if abs(last_position - position) <= tolerance:
                    return last_position
            except ServoTimeout:
                pass
            time.sleep(0.05)
        raise ServoTimeout(
            f"Servo {self.id} did not reach {position}; last present position was {last_position}"
        )

    def move_to_position(
        self,
        position: int,
        *,
        wait: bool = True,
        timeout: float = 30.0,
        tolerance: int = 32,
    ) -> int:
        status = self._movement_status()
        return self._command_position(
            position, status, wait=wait, timeout=timeout, tolerance=tolerance
        )

    def move_to_angle(
        self,
        angle_degrees: float,
        *,
        wait: bool = True,
        timeout: float = 30.0,
        tolerance: int = 32,
    ) -> int:
        status = self._movement_status()
        target = self.angle_to_position(
            angle_degrees, status.minimum_position, status.maximum_position
        )
        self._command_position(
            target, status, wait=wait, timeout=timeout, tolerance=tolerance
        )
        return target
