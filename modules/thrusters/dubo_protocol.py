"""DUBO / Hydrion-Spectra ESP32 binary UART protocol.

Implements the exact framing + CRC used by the ESP32 thruster-controller
firmware. Pi-side (this host) acts as the commander:

    AA 55 | VERSION 0x01 | TYPE | LENGTH | SEQ u16 LE | PAYLOAD | CRC16 u16 LE

CRC16-CCITT-FALSE covers the COMPLETE frame except the CRC itself
(polynomial 0x1021, init 0xFFFF, no reflect, XOR-out 0x0000).
"""

CRC_TABLE = None


def _crc_table():
    global CRC_TABLE
    if CRC_TABLE is None:
        table = []
        for value in range(256):
            crc = value << 8
            for _ in range(8):
                crc = ((crc << 1) ^ 0x1021) if (crc & 0x8000) else (crc << 1)
                crc &= 0xFFFF
            table.append(crc)
        CRC_TABLE = table
    return CRC_TABLE


def crc16_ccitt_false(data):
    """CRC16-CCITT-FALSE over `data` (bytes) as uint16."""
    crc = 0xFFFF
    for byte in data:
        crc = ((crc << 8) & 0xFFFF) ^ _crc_table()[((crc >> 8) ^ byte) & 0xFF]
    return crc & 0xFFFF


HEADER_1 = 0xAA
HEADER_2 = 0x55
PROTOCOL_VERSION = 0x01

MSG_MOTOR_COMMAND = 0x01
MSG_ESTOP = 0x02
MSG_ACK = 0x81
MSG_STATUS = 0x82

ACK_OK = 0
ACK_CRC_ERROR = 1
ACK_BAD_LENGTH = 2
ACK_BAD_RANGE = 3
ACK_BAD_FLAGS = 4
ACK_OLD_SEQUENCE = 5
ACK_UNKNOWN_TYPE = 6
ACK_ESTOP_ACTIVE = 7

ACK_NAMES = {
    ACK_OK: "OK",
    ACK_CRC_ERROR: "CRC_ERROR",
    ACK_BAD_LENGTH: "BAD_LENGTH",
    ACK_BAD_RANGE: "BAD_RANGE",
    ACK_BAD_FLAGS: "BAD_FLAGS",
    ACK_OLD_SEQUENCE: "OLD_SEQUENCE",
    ACK_UNKNOWN_TYPE: "UNKNOWN_TYPE",
    ACK_ESTOP_ACTIVE: "ESTOP_ACTIVE",
}

MOTOR_COMMAND_LENGTH = 13
ACK_LENGTH = 3
STATUS_LENGTH = 14
ESTOP_LENGTH = 0

MAX_PAYLOAD_LENGTH = 32
MAX_FRAME_SIZE = 41

MOTOR_NAME_TO_INDEX = {
    "M1": 0,
    "M2": 1,
    "M3": 2,
    "M4": 3,
    "M5": 4,
}


def build_frame(msg_type, sequence, payload=b""):
    """Pack a complete frame (header + type + length + seq + payload + CRC)."""
    if len(payload) > MAX_PAYLOAD_LENGTH:
        raise ValueError("payload too large")
    payload = bytes(payload)
    header = bytes(
        [HEADER_1, HEADER_2, PROTOCOL_VERSION, msg_type, len(payload)]
    )
    body = header + (sequence & 0xFFFF).to_bytes(2, "little") + payload
    crc = crc16_ccitt_false(body)
    return body + crc.to_bytes(2, "little")


def build_motor_command(sequence, m1, m2, m3, m4, m5, flags=0):
    """Motor command frame.

    Values must be int16 in the normalized -1000..+1000 range.
    Order byte matches ESP32: [0..1]=M1, [2..3]=M2, [4..5]=M3,
    [6..7]=M4, [8..9]=M5, [10]=FLAGS, [11..12]=RESERVED.
    """
    payload = bytearray(MOTOR_COMMAND_LENGTH)
    for offset, value in enumerate((m1, m2, m3, m4, m5)):
        payload[offset * 2 : offset * 2 + 2] = int(value).to_bytes(2, "little", signed=True)
    payload[10] = flags & 0x03
    payload[11] = 0
    payload[12] = 0
    return build_frame(MSG_MOTOR_COMMAND, sequence, payload)


def build_estop(sequence):
    """E-STOP frame (type 0x02, zero-length payload). Latches on the ESP32."""
    return build_frame(MSG_ESTOP, sequence, b"")


def parse_frame(data):
    """Parse a single complete frame.

    Returns (msg_type, sequence, payload) or None if the frame is
    not valid (bad header/version/length/CRC).
    """
    if len(data) < 9:
        return None
    if data[0] != HEADER_1 or data[1] != HEADER_2:
        return None
    if data[2] != PROTOCOL_VERSION:
        return None
    msg_type = data[3]
    length = data[4]
    expected = 5 + 2 + length + 2
    if len(data) != expected or length > MAX_PAYLOAD_LENGTH:
        return None
    received_crc = int.from_bytes(data[expected - 2 :], "little")
    if crc16_ccitt_false(data[: expected - 2]) != received_crc:
        return None
    sequence = int.from_bytes(data[5:7], "little")
    payload = data[7 : 7 + length]
    return (msg_type, sequence, payload)


def parse_status(payload):
    """Decode a 14-byte STATUS payload into a dict."""
    if len(payload) != STATUS_LENGTH:
        return None
    return {
        "sequence": int.from_bytes(payload[0:2], "little"),
        "failsafe": bool(payload[2]),
        "estop": bool(payload[3]),
        "motors": {
            name: int.from_bytes(payload[offset : offset + 2], "little", signed=True)
            for name, offset in (("M1", 4), ("M2", 6), ("M3", 8), ("M4", 10), ("M5", 12))
        },
    }


def parse_ack(payload):
    """Decode a 3-byte ACK payload into (status, echoed_sequence)."""
    if len(payload) != ACK_LENGTH:
        return None
    return (payload[0], int.from_bytes(payload[1:3], "little"))
