"""Pure observation gates; this module cannot publish commands or actions."""
import math


def check_zero_command(axes):
    if len(axes) != 6 or any(not math.isfinite(v) or v != 0. for v in axes):
        raise ValueError('nonzero_or_invalid_command_during_stationary_capture')


def source_fresh(stamp, sim, *, max_age):
    return (stamp is not None and all(math.isfinite(v) for v in (stamp, sim, max_age))
            and max_age > 0 and 0 <= sim-stamp <= max_age)


def capture_ready(map_stamps, sim, *, first_observation_sim, required_duration=30.):
    if not math.isfinite(sim) or required_duration < 30.:
        raise ValueError('Invalid observation duration/clock')
    if any(not math.isfinite(v) for v in map_stamps) or any(
            b < a for a, b in zip(map_stamps, map_stamps[1:])):
        raise ValueError('structural_clock_regression')
    distinct = list(dict.fromkeys(map_stamps))
    return (first_observation_sim is not None and math.isfinite(first_observation_sim)
            and len(distinct) >= 3 and sim >= first_observation_sim + required_duration
            and source_fresh(distinct[-1], sim, max_age=7.5))
