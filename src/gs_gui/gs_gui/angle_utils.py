import math


TWO_PI = 2.0 * math.pi


def normalize_rad_0_2pi(angle_rad: float) -> float:
    """Normalize radians to [0, 2π)."""
    return angle_rad % TWO_PI


def normalize_deg_0_360(angle_deg: float) -> float:
    """Normalize degrees to [0, 360)."""
    return angle_deg % 360.0


def rad_to_normalized_deg_0_360(angle_rad: float) -> float:
    """Convert radians to degrees and normalize to [0, 360)."""
    return normalize_deg_0_360(math.degrees(angle_rad))
