import math
import struct
from pathlib import Path

import pytest

from qcar_firmware import FIELDS, PROFILES, MAX_CHANNELS
from qcar_protocol import Protocol, slider_command


@pytest.mark.parametrize('profile,key,expected',[
    ('startup','fault_reason',4),
    ('dual','dual_pitch_pwm',-3000),
    ('angle','balance_actual_deg',1.7),
])
def test_actual_firmware_frames(profile,key,expected):
    protocol=Protocol()
    protocol.feed(sub(PROFILES[profile]))
    data=(Path(__file__).parent/'fixtures'/('qcar_'+profile+'.bin')).read_bytes()
    values=protocol.feed(data)['values']
    assert tuple(values)==tuple(PROFILES[profile])
    assert values[key]==pytest.approx(expected)


def sub(names):
    return ("MCAR SUB " + ",".join(names) + "\n").encode("ascii")


def frame(values):
    return struct.pack("<%df" % len(values), *values) + b"\x00\x00\x80\x7f"


@pytest.mark.parametrize("profile", list(PROFILES))
def test_schema_profile_order(profile):
    names = PROFILES[profile]
    protocol = Protocol()
    result = protocol.feed(sub(names))
    assert result == {"kind": "schema", "fields": tuple(names)}
    assert isinstance(result["fields"], tuple)
    assert protocol.fields == tuple(names)


def test_subscription_limits_and_validation():
    protocol = Protocol()
    result = protocol.feed(sub(FIELDS[:MAX_CHANNELS]))
    assert result["fields"] == tuple(FIELDS[:MAX_CHANNELS])

    old = protocol.fields
    for bad in (
        sub(FIELDS[:MAX_CHANNELS+1]),
        sub([FIELDS[0], FIELDS[0]]),
        sub(["not_a_field"]),
        sub([""]),
        b"MCAR SUB roll_deg,\n",
        b"MCAR SUB roll_deg pitch_deg\n",
        b"MCAR SUB\n",
        b"MCAR SUB " + b"a" * 30 + b"\n",
    ):
        with pytest.raises(ValueError):
            protocol.feed(bad)
        assert protocol.fields == old

    assert protocol.feed(sub(["pitch_deg"])) == {
        "kind": "schema",
        "fields": ("pitch_deg",),
    }


def test_subscription_change_and_case_strictness():
    protocol = Protocol()
    protocol.feed(sub(["roll_deg", "pitch_deg"]))
    protocol.feed(sub(["yaw_deg"]))
    assert protocol.fields == ("yaw_deg",)
    with pytest.raises(ValueError):
        protocol.feed(b"MCAR sub roll_deg\n")
    assert protocol.fields == ("yaw_deg",)


def test_rate_and_stream_ack_and_atomicity():
    protocol = Protocol()
    assert protocol.feed(b"MCAR RATE 2\n") == {"kind": "rate", "value": 2}
    assert protocol.period_ms == 2
    assert protocol.feed(b"MCAR RATE 1000\n") == {"kind": "rate", "value": 1000}
    assert protocol.period_ms == 1000
    for bad in (b"MCAR RATE 1\n", b"MCAR RATE 1001\n", b"MCAR RATE 2.5\n", b"MCAR RATE x\n"):
        with pytest.raises(ValueError):
            protocol.feed(bad)
        assert protocol.period_ms == 1000

    assert protocol.feed(b"MCAR STREAM 0\n") == {"kind": "stream", "value": 0}
    assert protocol.stream == 0
    assert protocol.feed(b"MCAR STREAM 1\n") == {"kind": "stream", "value": 1}
    assert protocol.stream == 1
    for bad in (b"MCAR STREAM 2\n", b"MCAR STREAM -1\n", b"MCAR STREAM true\n"):
        with pytest.raises(ValueError):
            protocol.feed(bad)
        assert protocol.stream == 1


def test_slider_list_error_ack():
    protocol = Protocol()
    assert protocol.feed(b"MCAR SLIDER rate_kp 12.5\n") == {
        "kind": "slider",
        "name": "rate_kp",
        "value": 12.5,
    }
    assert protocol.feed(b"MCAR SLIDER balance_angle_x -3.25\n") == {
        "kind": "slider",
        "name": "balance_angle_x",
        "value": -3.25,
    }
    for bad in (
        b"MCAR SLIDER rate_kp nan\n",
        b"MCAR SLIDER rate_kp inf\n",
        b"MCAR SLIDER rate_kp\n",
        b"MCAR SLIDER rate_kp abc\n",
    ):
        with pytest.raises(ValueError):
            protocol.feed(bad)

    assert protocol.feed(b"MCAR LIST roll_deg,pitch_deg\n") == {
        "kind": "list",
        "fields": ("roll_deg", "pitch_deg"),
    }
    assert protocol.feed(b"MCAR LIST roll_deg\n")["fields"] == ("roll_deg",)
    assert protocol.feed(b"MCAR ERR boom\n") == {"kind": "error", "text": "boom"}
    assert protocol.feed(b"MCAR ERROR bad things\n") == {
        "kind": "error",
        "text": "bad things",
    }
    assert protocol.feed(b"MCAR ERR\n") == {"kind": "error", "text": ""}
    assert protocol.feed(b"MCAR ERROR\n") == {"kind": "error", "text": ""}


def test_unknown_and_multiline_text_rejected():
    protocol = Protocol()
    for bad in (
        b"MCAR HELLO\n",
        b"MCAR LIST \n",
        b"MCAR SUB \n",
        b"HELLO\n",
        b"MCAR STREAM 1\nMCAR STREAM 0\n",
    ):
        with pytest.raises(ValueError):
            protocol.feed(bad)
    assert protocol.stream is None


def test_binary_data_and_errors():
    protocol = Protocol()
    with pytest.raises(ValueError):
        protocol.feed(frame([1.0, 2.0]))
    assert protocol.fields == ()

    protocol.feed(sub(["roll_deg", "pitch_deg"]))
    result = protocol.feed(frame([1.5, -2.25]))
    assert result["kind"] == "data"
    assert list(result["values"]) == ["roll_deg", "pitch_deg"]
    assert result["values"]["roll_deg"] == pytest.approx(1.5)
    assert result["values"]["pitch_deg"] == pytest.approx(-2.25)

    values = result["values"]
    for bad in (
        frame([1.5]),
        frame([1.5, -2.25, 3.0]),
        frame([1.5, -2.25])[:-1] + b"\x00",
        frame([1.5, float("nan")]),
        frame([float("inf"), -2.25]),
        frame([float("-inf"), -2.25]),
    ):
        with pytest.raises(ValueError):
            protocol.feed(bad)
    assert protocol.fields == ("roll_deg", "pitch_deg")
    assert protocol.feed(frame([4.0, 5.0]))["values"] == {
        "roll_deg": pytest.approx(4.0),
        "pitch_deg": pytest.approx(5.0),
    }
    # A valid frame is not damaged by a prior rejection.
    assert values["roll_deg"] == pytest.approx(1.5)


def test_slider_command_exact_output_and_format():
    assert slider_command("rate_kp", 0.1234567890123) == (
        b"[slider,rate_kp,0.123456789]\n"
    )
    assert slider_command("rate_sign", 1.0) == b"[slider,rate_sign,1]\n"
    assert slider_command("rate_sign", -1) == b"[slider,rate_sign,-1]\n"
    assert slider_command("balance_angle_x", -20) == (
        b"[slider,balance_angle_x,-20]\n"
    )
    assert slider_command("rate_pwm_limit", 4000) == b"[slider,rate_pwm_limit,4000]\n"


def test_slider_command_ranges_and_rejections():
    assert slider_command("rate_kp", 0)
    assert slider_command("rate_kp", 500)
    assert slider_command("rate_pwm_limit", 0)
    assert slider_command("rate_pwm_limit", 4000)
    assert slider_command("balance_angle_y", 20)
    assert slider_command("balance_angle_y", -20)

    for name, bad_values in (
        ("rate_kp", (-0.001, 500.001)),
        ("rate_pwm_limit", (-1, 4001)),
        ("rate_sign", (0, 2, -2, 1.0001)),
        ("balance_angle_x", (-20.001, 20.001)),
        ("balance_angle_y", (-21, 21)),
    ):
        for value in bad_values:
            with pytest.raises(ValueError):
                slider_command(name, value)

    for name in ("rate_kp", "rate_sign", "balance_angle_x"):
        for value in (True, False, float("nan"), float("inf"), float("-inf"), "1"):
            with pytest.raises(ValueError):
                slider_command(name, value)
    with pytest.raises(ValueError):
        slider_command("unknown", 1.0)
    with pytest.raises(ValueError):
        slider_command("RATE_KP", 1.0)
