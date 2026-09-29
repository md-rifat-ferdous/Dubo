from copy import deepcopy
from pathlib import Path

import yaml

MOTOR_NAMES = ("M1", "M2", "M3", "M4", "M5")
AXIS_NAMES = ("surge", "sway", "heave", "yaw", "pitch", "roll")
PROJECT_ROOT = Path(__file__).resolve().parents[2]

DEFAULT_MIXING = {
    "M1": {"heave": 1.0, "pitch": -1.0},
    "M2": {"surge": 1.0, "yaw": -1.0},
    "M3": {"surge": 1.0, "yaw": 1.0},
    "M4": {"heave": 1.0, "pitch": 1.0, "roll": 1.0},
    "M5": {"heave": 1.0, "pitch": 1.0, "roll": -1.0},
}

DEFAULT_POLARITY = {name: 1 for name in MOTOR_NAMES}


def resolve_config_path(path=None):
    value = Path(path) if path else PROJECT_ROOT / "configs" / "thrusters.yaml"
    if not value.is_absolute():
        value = PROJECT_ROOT / value
    return value


def normalize_mixing(value=None):
    result = deepcopy(DEFAULT_MIXING)
    if not value:
        return result
    if not isinstance(value, dict):
        raise ValueError("mixing must be a mapping")
    for name, weights in value.items():
        if name not in DEFAULT_MIXING:
            raise ValueError(f"unknown motor in mixing: {name}")
        if not isinstance(weights, dict):
            raise ValueError(f"mixing for {name} must be a mapping")
        clean = {}
        for axis, weight in weights.items():
            if axis not in AXIS_NAMES:
                raise ValueError(f"unknown motion axis for {name}: {axis}")
            clean[axis] = float(weight)
        result[name] = clean
    return result


def normalize_polarity(value=None):
    result = dict(DEFAULT_POLARITY)
    if not value:
        return result
    if not isinstance(value, dict):
        raise ValueError("polarity must be a mapping")
    for name, direction in value.items():
        if name not in DEFAULT_POLARITY:
            raise ValueError(f"unknown motor in polarity: {name}")
        direction = int(direction)
        if direction not in (-1, 1):
            raise ValueError(f"polarity for {name} must be -1 or 1")
        result[name] = direction
    return result


def normalize_motor_values(value=None):
    result = {name: 0.0 for name in MOTOR_NAMES}
    if not value:
        return result
    if not isinstance(value, dict):
        raise ValueError("motor values must be a mapping")
    for name, amount in value.items():
        if name not in result:
            raise ValueError(f"unknown motor value: {name}")
        result[name] = max(-1.0, min(1.0, float(amount)))
    return result


def mix_motion(motion, mixing=None, polarity=None, boost_scale=1.0):
    mixing = normalize_mixing(mixing)
    polarity = normalize_polarity(polarity)
    scale = max(0.0, min(1.0, float(boost_scale))) if getattr(motion, "boost", False) else 1.0
    result = {}
    for name in MOTOR_NAMES:
        weights = mixing.get(name, {})
        value = sum(getattr(motion, axis, 0.0) * weight for axis, weight in weights.items())
        value *= polarity.get(name, 1)
        result[name] = max(-1.0, min(1.0, value * scale))
    return result


def load_thruster_config(path=None):
    config_path = resolve_config_path(path)
    try:
        with config_path.open() as stream:
            data = yaml.safe_load(stream) or {}
    except (OSError, yaml.YAMLError):
        data = {}
    if isinstance(data, dict) and "thrusters" in data:
        data = data.get("thrusters") or {}
    if not isinstance(data, dict):
        data = {}
    result = dict(data)
    result["mixing"] = normalize_mixing(result.get("mixing"))
    result["polarity"] = normalize_polarity(result.get("polarity"))
    deadband = result.get("deadband") or {}
    result["deadband"] = {
        "positive": max(0, int(deadband.get("positive", 20))),
        "negative": max(0, int(deadband.get("negative", 176))),
    }
    result.setdefault("port", "/dev/ttyUSB0")
    result.setdefault("baud", 115200)
    result.setdefault("ramp", 0.1)
    result.setdefault("boost_scale", 1.0)
    return result


def load_thruster_deadband(path=None):
    config = load_thruster_config(path)
    deadband = config["deadband"]
    return deadband["positive"], -abs(deadband["negative"])
