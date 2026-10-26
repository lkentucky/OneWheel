"""Pure-Python parser for the QCar monitor protocol.

Constants are intentionally read from :mod:`qcar_firmware`; no firmware
configuration lives in this module.
"""

import math
import struct

from qcar_firmware import FIELDS, MAX_CHANNELS
from qcar_parameters import SLIDER_LIMITS, SIGN_PARAMETERS


_TRAILER = b"\x00\x00\x80\x7f"
_FIELD_SET = frozenset(FIELDS)

_SLIDER_LIMITS = SLIDER_LIMITS
_SLIDER_NAMES = frozenset(_SLIDER_LIMITS) | SIGN_PARAMETERS


def _reject(message):
    raise ValueError(message)


class Protocol:
    """Incremental line/frame parser for the QCar monitor protocol."""

    fields = ()
    period_ms = None
    stream = None

    def __init__(self):
        self.fields = ()
        self.period_ms = None
        self.stream = None

    def feed(self, data):
        """Parse exactly one control line or one binary telemetry frame."""
        if not isinstance(data, (bytes, bytearray, memoryview)):
            _reject("data must be bytes-like")
        raw = bytes(data)
        if self.fields:
            expected = len(self.fields) * 4 + 4
            if len(raw) == expected and raw[-4:] == _TRAILER:
                return self._feed_binary(raw)
        if raw.startswith(b"MCAR "):
            return self._feed_text(raw)
        return self._feed_binary(raw)

    def _feed_text(self, raw):
        try:
            text = raw.decode("ascii")
        except UnicodeDecodeError:
            _reject("control line is not ASCII")

        if text.endswith("\n"):
            text = text[:-1]
            if text.endswith("\r"):
                text = text[:-1]
        if "\n" in text or "\r" in text:
            _reject("feed accepts one line at a time")

        if text.startswith("MCAR SUB "):
            return self._subscribe(text[len("MCAR SUB "):])
        if text.startswith("MCAR RATE "):
            return self._rate(text[len("MCAR RATE "):])
        if text.startswith("MCAR STREAM "):
            return self._stream(text[len("MCAR STREAM "):])
        if text.startswith("MCAR SLIDER "):
            return self._slider(text[len("MCAR SLIDER "):])
        if text.startswith("MCAR LIST "):
            return self._list(text[len("MCAR LIST "):])

        for prefix in ("MCAR ERROR", "MCAR ERR"):
            if text == prefix:
                return {"kind": "error", "text": ""}
            if text.startswith(prefix + " "):
                return {"kind": "error", "text": text[len(prefix) + 1:].strip()}
        _reject("unknown control line")

    def _subscribe(self, payload):
        names = payload.split(",")
        if not 1 <= len(names) <= MAX_CHANNELS:
            _reject(f"subscription must contain 1..{MAX_CHANNELS} fields")
        if len(set(names)) != len(names):
            _reject("subscription contains duplicate fields")
        if any(not name or name not in _FIELD_SET for name in names):
            _reject("subscription contains an unknown field")
        self.fields = tuple(names)
        return {"kind": "schema", "fields": self.fields}

    def _rate(self, payload):
        if not payload.isdigit():
            _reject("rate must be an integer")
        value = int(payload)
        if not 2 <= value <= 1000:
            _reject("rate out of range")
        self.period_ms = value
        return {"kind": "rate", "value": value}

    def _stream(self, payload):
        if payload not in ("0", "1"):
            _reject("stream must be 0 or 1")
        value = int(payload)
        self.stream = value
        return {"kind": "stream", "value": value}

    def _slider(self, payload):
        name, sep, value_text = payload.partition(" ")
        if (
            not sep
            or not name
            or not value_text
            or name.strip() != name
            or value_text.strip() != value_text
        ):
            _reject("malformed slider reply")
        try:
            value = float(value_text)
        except ValueError:
            _reject("slider value is not a float")
        if not math.isfinite(value):
            _reject("slider value is not finite")
        return {"kind": "slider", "name": name, "value": value}

    def _list(self, payload):
        names = payload.split(",")
        if any(not name or name.strip() != name for name in names):
            _reject("malformed list reply")
        return {"kind": "list", "fields": tuple(names)}

    def _feed_binary(self, raw):
        if not self.fields:
            _reject("binary data before SUB")
        expected = len(self.fields) * 4 + 4
        if len(raw) != expected:
            _reject("binary frame has the wrong length")
        if raw[-4:] != _TRAILER:
            _reject("binary frame has the wrong trailer")
        values = struct.unpack("<%df" % len(self.fields), raw[:-4])
        if any(not math.isfinite(value) for value in values):
            _reject("binary frame contains a non-finite value")
        return {
            "kind": "data",
            "values": dict(zip(self.fields, values)),
        }


def slider_command(name, value):
    """Build an ASCII ``[slider,name,value]`` command for a test parameter."""
    if not isinstance(name, str) or name not in _SLIDER_NAMES:
        _reject("unknown slider name")
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        _reject("slider value must be a number")
    try:
        number = float(value)
    except (OverflowError, ValueError):
        _reject("slider value must be a number")
    if not math.isfinite(number):
        _reject("slider value must be finite")

    if name in SIGN_PARAMETERS:
        if number not in (-1.0, 1.0):
            _reject(name + " must be -1 or 1")
    else:
        low, high = _SLIDER_LIMITS[name]
        if not low <= number <= high:
            _reject("slider value out of range")

    return ("[slider,%s,%s]\n" % (name, format(number, ".9g"))).encode("ascii")
