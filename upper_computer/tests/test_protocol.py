from __future__ import annotations

from pathlib import Path
import struct
import unittest

from sailing.models import ControlCommand, FireRequest, FireSource
from sailing.protocol import (
    Frame,
    FrameStreamDecoder,
    MAX_PAYLOAD,
    MessageType,
    ProtocolError,
    crc16_ccitt,
    decode_control_payload,
    decode_fire_payload,
    decode_frame,
    encode_control_payload,
    encode_fire_payload,
    encode_frame,
)


def load_vectors() -> dict[str, tuple[bool, bytes]]:
    path = Path(__file__).resolve().parents[2] / "shared" / "test-vectors" / "protocol-v1.tsv"
    vectors: dict[str, tuple[bool, bytes]] = {}
    for line in path.read_text(encoding="utf-8").splitlines():
        if not line or line.startswith("#"):
            continue
        name, validity, hexadecimal = line.split("\t")
        vectors[name] = validity == "true", bytes.fromhex(hexadecimal)
    return vectors


class ProtocolTests(unittest.TestCase):
    def test_crc_known_check_value(self) -> None:
        self.assertEqual(crc16_ccitt(b"123456789"), 0x29B1)

    def test_shared_golden_vectors_decode(self) -> None:
        vectors = load_vectors()
        self.assertEqual(set(vectors), {"control", "estop", "fire", "bad_crc"})
        for name, (valid, encoded) in vectors.items():
            if valid:
                self.assertIsInstance(decode_frame(encoded), Frame, name)
            else:
                with self.assertRaises(ProtocolError, msg=name):
                    decode_frame(encoded)

    def test_control_vector_is_generated_from_fields(self) -> None:
        expected = load_vectors()["control"][1]
        actual = encode_frame(
            Frame(
                message_type=MessageType.CONTROL_SETPOINT,
                session_id=0x11223344,
                sequence=0x01020304,
                sender_time_ms=123456,
                payload=encode_control_payload(ControlCommand(0.5, -0.25), valid_for_ms=500),
            )
        )
        self.assertEqual(actual, expected)
        decoded = decode_frame(actual)
        command, valid_for_ms = decode_control_payload(decoded.payload)
        self.assertAlmostEqual(command.propulsion, 0.5)
        self.assertAlmostEqual(command.steering, -0.25)
        self.assertEqual(valid_for_ms, 500)

    def test_fire_payload_round_trip(self) -> None:
        request = FireRequest(shot_id=9, source=FireSource.AUTO, valid_for_ms=250)
        self.assertEqual(decode_fire_payload(encode_fire_payload(request)), request)

    def test_stream_decoder_handles_noise_chunks_and_corruption(self) -> None:
        vectors = load_vectors()
        decoder = FrameStreamDecoder()
        stream = b"noise" + vectors["bad_crc"][1] + vectors["estop"][1] + vectors["fire"][1]
        decoded: list[Frame] = []
        for offset in range(0, len(stream), 3):
            decoded.extend(decoder.feed(stream[offset : offset + 3]))
        self.assertEqual([frame.message_type for frame in decoded], [MessageType.E_STOP, MessageType.FIRE_ONCE])
        self.assertGreaterEqual(decoder.rejected_frames, 1)
        self.assertEqual(decoder.buffered_bytes, 0)

    def test_stream_decoder_keeps_partial_magic(self) -> None:
        decoder = FrameStreamDecoder()
        self.assertEqual(decoder.feed(b"junkS"), [])
        self.assertEqual(decoder.buffered_bytes, 1)
        frame = load_vectors()["estop"][1]
        decoded = decoder.feed(frame[1:])
        self.assertEqual(len(decoded), 1)

    def test_rejects_invalid_frame_fields(self) -> None:
        base = dict(
            message_type=MessageType.HEARTBEAT,
            session_id=1,
            sequence=1,
            sender_time_ms=1,
        )
        with self.assertRaises(ProtocolError):
            encode_frame(Frame(**base, flags=1))
        with self.assertRaises(ProtocolError):
            encode_frame(Frame(**base, payload=b"x" * (MAX_PAYLOAD + 1)))
        with self.assertRaises(ProtocolError):
            encode_frame(Frame(**base, version=2))
        with self.assertRaises(ProtocolError):
            decode_frame(b"short")

    def test_rejects_bad_control_and_fire_payloads(self) -> None:
        with self.assertRaises(ValueError):
            ControlCommand(float("inf"), 0.0)
        with self.assertRaises(ProtocolError):
            encode_control_payload(ControlCommand(1.1, 0.0))
        with self.assertRaises(ProtocolError):
            encode_control_payload(ControlCommand(), valid_for_ms=0)
        with self.assertRaises(ProtocolError):
            decode_control_payload(struct.pack("<ffH", float("nan"), 0.0, 1))
        with self.assertRaises(ProtocolError):
            decode_control_payload(b"bad")
        with self.assertRaises(ProtocolError):
            decode_fire_payload(struct.pack("<IBH", 1, 99, 250))
        with self.assertRaises(ProtocolError):
            decode_fire_payload(b"bad")


if __name__ == "__main__":
    unittest.main()
