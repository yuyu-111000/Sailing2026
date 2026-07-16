"""Protocol-v1 framing shared with the lower-computer C++17 core."""

from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum
import math
import struct

from .models import ControlCommand, FireRequest, FireSource

MAGIC = b"SA"
PROTOCOL_VERSION = 1
MAX_PAYLOAD = 256
_HEADER_STRUCT = struct.Struct("<BBBIIHI")
_FIXED_PREFIX_SIZE = len(MAGIC) + _HEADER_STRUCT.size
_CRC_SIZE = 2
MAX_FRAME_SIZE = _FIXED_PREFIX_SIZE + MAX_PAYLOAD + _CRC_SIZE
_CONTROL_STRUCT = struct.Struct("<ffH")
_FIRE_STRUCT = struct.Struct("<IBH")


class ProtocolError(ValueError):
    """Raised when an encoded frame violates protocol v1."""


class MessageType(IntEnum):
    HEARTBEAT = 0x01
    ARM = 0x02
    DISARM = 0x03
    E_STOP = 0x04
    CONTROL_SETPOINT = 0x05
    MISSION_STATUS = 0x06
    TELEMETRY = 0x07
    LAUNCH_ARM = 0x08
    FIRE_ONCE = 0x09
    ACK = 0x0A
    NACK = 0x0B
    FAULT = 0x0C


@dataclass(frozen=True, slots=True)
class Frame:
    """One decoded protocol message."""

    message_type: MessageType
    session_id: int
    sequence: int
    sender_time_ms: int
    payload: bytes = b""
    flags: int = 0
    version: int = PROTOCOL_VERSION


def crc16_ccitt(data: bytes) -> int:
    """Compute CRC-16/CCITT-FALSE."""

    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def _as_message_type(value: MessageType | int) -> MessageType:
    try:
        return MessageType(value)
    except ValueError as error:
        raise ProtocolError(f"unsupported message type {value}") from error


def encode_frame(frame: Frame) -> bytes:
    """Validate and encode one protocol frame."""

    if frame.version != PROTOCOL_VERSION:
        raise ProtocolError(f"unsupported protocol version {frame.version}")
    message_type = _as_message_type(frame.message_type)
    if frame.flags != 0:
        raise ProtocolError("protocol v1 defines no non-zero frame flags")
    if not 0 <= frame.session_id <= 0xFFFFFFFF:
        raise ProtocolError("session_id must fit in uint32")
    if not 0 <= frame.sequence <= 0xFFFFFFFF:
        raise ProtocolError("sequence must fit in uint32")
    if not 0 <= frame.sender_time_ms <= 0xFFFFFFFF:
        raise ProtocolError("sender_time_ms must fit in uint32")
    if len(frame.payload) > MAX_PAYLOAD:
        raise ProtocolError(f"payload exceeds {MAX_PAYLOAD} bytes")

    body = _HEADER_STRUCT.pack(
        frame.version,
        int(message_type),
        frame.flags,
        frame.session_id,
        frame.sequence,
        len(frame.payload),
        frame.sender_time_ms,
    ) + frame.payload
    return MAGIC + body + struct.pack("<H", crc16_ccitt(body))


def decode_frame(data: bytes) -> Frame:
    """Decode exactly one complete frame, with no actuator side effects."""

    if len(data) < _FIXED_PREFIX_SIZE + _CRC_SIZE:
        raise ProtocolError("frame is incomplete")
    if data[:2] != MAGIC:
        raise ProtocolError("invalid magic")

    version, raw_type, flags, session_id, sequence, payload_length, sender_time_ms = (
        _HEADER_STRUCT.unpack(data[2:_FIXED_PREFIX_SIZE])
    )
    if version != PROTOCOL_VERSION:
        raise ProtocolError(f"unsupported protocol version {version}")
    message_type = _as_message_type(raw_type)
    if flags != 0:
        raise ProtocolError("protocol v1 defines no non-zero frame flags")
    if payload_length > MAX_PAYLOAD:
        raise ProtocolError("payload length exceeds limit")
    expected_length = _FIXED_PREFIX_SIZE + payload_length + _CRC_SIZE
    if len(data) != expected_length:
        raise ProtocolError("frame length does not match payload length")

    body = data[2:-2]
    expected_crc = struct.unpack("<H", data[-2:])[0]
    if crc16_ccitt(body) != expected_crc:
        raise ProtocolError("CRC mismatch")

    return Frame(
        message_type=message_type,
        session_id=session_id,
        sequence=sequence,
        sender_time_ms=sender_time_ms,
        payload=bytes(data[_FIXED_PREFIX_SIZE:-2]),
        flags=flags,
        version=version,
    )


class FrameStreamDecoder:
    """Bounded incremental decoder that resynchronizes after bad input."""

    def __init__(self) -> None:
        self._buffer = bytearray()
        self.rejected_frames = 0
        self.dropped_bytes = 0

    @property
    def buffered_bytes(self) -> int:
        return len(self._buffer)

    def feed(self, data: bytes) -> list[Frame]:
        """Append a chunk and return all newly completed valid frames."""

        self._buffer.extend(data)
        frames: list[Frame] = []
        while True:
            magic_index = self._buffer.find(MAGIC)
            if magic_index < 0:
                keep = 1 if self._buffer[-1:] == MAGIC[:1] else 0
                self.dropped_bytes += len(self._buffer) - keep
                if keep:
                    del self._buffer[:-1]
                else:
                    self._buffer.clear()
                break
            if magic_index:
                self.dropped_bytes += magic_index
                del self._buffer[:magic_index]
            if len(self._buffer) < _FIXED_PREFIX_SIZE:
                break

            payload_length = struct.unpack("<H", self._buffer[13:15])[0]
            if payload_length > MAX_PAYLOAD:
                self.rejected_frames += 1
                self.dropped_bytes += 1
                del self._buffer[0]
                continue
            frame_length = _FIXED_PREFIX_SIZE + payload_length + _CRC_SIZE
            if len(self._buffer) < frame_length:
                break
            candidate = bytes(self._buffer[:frame_length])
            try:
                frames.append(decode_frame(candidate))
                del self._buffer[:frame_length]
            except ProtocolError:
                self.rejected_frames += 1
                self.dropped_bytes += 1
                del self._buffer[0]
        if len(self._buffer) > MAX_FRAME_SIZE:
            excess = len(self._buffer) - MAX_FRAME_SIZE
            self.dropped_bytes += excess
            del self._buffer[:excess]
        return frames


def encode_control_payload(command: ControlCommand, *, valid_for_ms: int = 500) -> bytes:
    """Encode a finite, normalized, time-bounded motion setpoint."""

    if not 1 <= valid_for_ms <= 0xFFFF:
        raise ProtocolError("valid_for_ms must fit in non-zero uint16")
    if not (-1.0 <= command.propulsion <= 1.0 and -1.0 <= command.steering <= 1.0):
        raise ProtocolError("control values must be normalized before encoding")
    return _CONTROL_STRUCT.pack(command.propulsion, command.steering, valid_for_ms)


def decode_control_payload(payload: bytes) -> tuple[ControlCommand, int]:
    """Decode a control payload and reject invalid floats or ranges."""

    if len(payload) != _CONTROL_STRUCT.size:
        raise ProtocolError("control payload has invalid length")
    propulsion, steering, valid_for_ms = _CONTROL_STRUCT.unpack(payload)
    if not math.isfinite(propulsion) or not math.isfinite(steering):
        raise ProtocolError("control payload contains a non-finite value")
    if not (-1.0 <= propulsion <= 1.0 and -1.0 <= steering <= 1.0):
        raise ProtocolError("control payload is outside normalized bounds")
    if valid_for_ms == 0:
        raise ProtocolError("control payload has zero validity")
    return ControlCommand(propulsion, steering), valid_for_ms


def encode_fire_payload(request: FireRequest) -> bytes:
    """Encode one unique, expiring fake-launch request."""

    return _FIRE_STRUCT.pack(request.shot_id, request.source.value, request.valid_for_ms)


def decode_fire_payload(payload: bytes) -> FireRequest:
    """Decode one fake-launch request."""

    if len(payload) != _FIRE_STRUCT.size:
        raise ProtocolError("fire payload has invalid length")
    shot_id, raw_source, valid_for_ms = _FIRE_STRUCT.unpack(payload)
    try:
        source = FireSource(raw_source)
    except ValueError as error:
        raise ProtocolError(f"unsupported fire source {raw_source}") from error
    try:
        return FireRequest(shot_id=shot_id, source=source, valid_for_ms=valid_for_ms)
    except ValueError as error:
        raise ProtocolError(str(error)) from error
