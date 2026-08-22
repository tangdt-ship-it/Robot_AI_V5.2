#!/usr/bin/env python3
"""Host-side RobotLink V3 framing self-test; no embedded toolchain required."""
from __future__ import annotations


def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def encode(sequence: int, command: str) -> str:
    if not 0 <= sequence <= 0xFFFF:
        raise ValueError("sequence")
    body = f"RAI,3,{sequence},{command}"
    if "," not in command:
        body += ","
    return f"${body}*{crc16_ccitt(body.encode()):04X}\r\n"


def decode(frame: str) -> tuple[int, str, str]:
    if not frame.startswith("$") or "*" not in frame:
        raise ValueError("framing")
    raw = frame.rstrip("\r\n")
    body, crc_text = raw[1:].rsplit("*", 1)
    if len(crc_text) != 4 or int(crc_text, 16) != crc16_ccitt(body.encode()):
        raise ValueError("crc")
    fields = body.split(",", 4)
    if len(fields) != 5 or fields[0] != "RAI" or fields[1] != "3":
        raise ValueError("protocol")
    sequence = int(fields[2])
    if not 0 <= sequence <= 0xFFFF:
        raise ValueError("sequence")
    return sequence, fields[3], fields[4]


def sequence_is_forward(previous: int, current: int) -> bool:
    delta = (current - previous) & 0xFFFF
    return delta != 0 and delta < 0x8000


def main() -> None:
    expected = {
        (0, "PING"): "$RAI,3,0,PING,*FBC5\r\n",
        (1, "HELLO,PROTO,3"): "$RAI,3,1,HELLO,PROTO,3*7B48\r\n",
        (42, "MOVE,FWD,500,15"): "$RAI,3,42,MOVE,FWD,500,15*1875\r\n",
        (43, "STOP"): "$RAI,3,43,STOP,*5777\r\n",
        (44, "HB"): "$RAI,3,44,HB,*EDA6\r\n",
    }
    for key, wanted in expected.items():
        actual = encode(*key)
        assert actual == wanted, (key, actual, wanted)
        seq, msg_type, payload = decode(actual)
        assert seq == key[0]
        rebuilt = msg_type if payload == "" else f"{msg_type},{payload}"
        assert rebuilt == key[1]

    broken = expected[(42, "MOVE,FWD,500,15")].replace("500", "501")
    try:
        decode(broken)
    except ValueError as exc:
        assert str(exc) == "crc"
    else:
        raise AssertionError("corrupted command was accepted")

    assert sequence_is_forward(41, 42)
    assert not sequence_is_forward(42, 42)  # duplicate
    assert not sequence_is_forward(42, 41)  # replay/backwards
    assert sequence_is_forward(0xFFFF, 0)    # wrap
    print("PASS RobotLink V3 CRC/framing/sequence self-test")


if __name__ == "__main__":
    main()
