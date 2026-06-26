import struct

from esp32_bridge.protocol import (
    MSG_BOARD_IDENTITY,
    MSG_ESTOP,
    MSG_ESTOP_CLEAR,
    MSG_TRACTION_CMD,
    ROLE_ARM,
    ROLE_CHASSIS,
    BoardIdentity,
    ChassisEstopMirror,
    FrameParser,
    RoleRouteTable,
    build_frame,
    build_traction_cmd,
    parse_identity,
)


class FakeLink:
    pass


def test_frame_parser_resyncs_and_decodes_identity():
    identity_payload = struct.pack('<BBH', ROLE_CHASSIS, 1, 0x001D)
    frame = build_frame(MSG_BOARD_IDENTITY, identity_payload)
    corrupt = bytearray(build_frame(MSG_BOARD_IDENTITY, b'bad'))
    corrupt[-1] ^= 0xFF

    parser = FrameParser()
    parsed = parser.feed(b'noise' + bytes(corrupt) + b'\xAA') + parser.feed(frame)

    assert parsed == [(MSG_BOARD_IDENTITY, identity_payload)]
    identity = parse_identity(parsed[0][1])
    assert identity.role == ROLE_CHASSIS
    assert identity.protocol_version == 1
    assert identity.capabilities == 0x001D


def test_role_route_table_assigns_and_replaces_roles():
    routes = RoleRouteTable()
    chassis_a = FakeLink()
    chassis_b = FakeLink()
    arm = FakeLink()

    assert routes.assign(chassis_a, BoardIdentity(ROLE_CHASSIS, 1, 0)) is None
    assert routes.get(ROLE_CHASSIS) is chassis_a
    assert routes.assign(arm, BoardIdentity(ROLE_ARM, 1, 0)) is None
    assert routes.get(ROLE_ARM) is arm
    assert routes.assign(chassis_b, BoardIdentity(ROLE_CHASSIS, 1, 0)) is chassis_a
    assert routes.get(ROLE_CHASSIS) is chassis_b

    routes.clear_link(chassis_b)
    assert routes.get(ROLE_CHASSIS) is None
    assert routes.get(ROLE_ARM) is arm


def test_build_traction_cmd_roundtrips_and_clamps():
    parser = FrameParser()
    # In-range values round-trip through the wire format (×1000, little-endian).
    parsed = parser.feed(build_traction_cmd(0.5, -0.25, True))
    assert len(parsed) == 1
    msg_type, payload = parsed[0]
    assert msg_type == MSG_TRACTION_CMD
    left, right, enable = struct.unpack('<hhB', payload)
    assert (left, right, enable) == (500, -250, 1)

    # Out-of-range speeds clamp to ±1000; enable=False sends 0.
    _, payload = FrameParser().feed(build_traction_cmd(2.0, -3.0, False))[0]
    left, right, enable = struct.unpack('<hhB', payload)
    assert (left, right, enable) == (1000, -1000, 0)


def test_chassis_estop_mirror_emits_only_on_transitions():
    mirror = ChassisEstopMirror()
    parser = FrameParser()

    assert mirror.update(False) is None

    stop = mirror.update(True)
    assert parser.feed(stop) == [(MSG_ESTOP, b'')]
    assert mirror.update(True) is None

    clear = mirror.update(False)
    assert parser.feed(clear) == [(MSG_ESTOP_CLEAR, b'')]
    assert mirror.update(False) is None


def test_chassis_estop_mirror_reset_releases_only_when_active():
    mirror = ChassisEstopMirror()
    parser = FrameParser()

    # Reset is a no-op when the chassis was not holding the arm e-stopped (so a
    # vanished chassis that was never in e-stop sends nothing to the arm).
    assert mirror.reset() is None

    # Chassis goes into e-stop, then its link disappears: reset clears the mirror
    # and yields a clear frame so the bridge can release the arm.
    mirror.update(True)
    clear = mirror.reset()
    assert parser.feed(clear) == [(MSG_ESTOP_CLEAR, b'')]
    assert mirror.active is False
    # Already cleared → a second reset is a no-op.
    assert mirror.reset() is None
    # A chassis that reconnects and re-asserts e-stop transitions cleanly again.
    assert parser.feed(mirror.update(True)) == [(MSG_ESTOP, b'')]
