import math


def wrap_180(angle_deg):
    return (
        (angle_deg + 180.0)
        % 360.0
    ) - 180.0


def valid_angle(value):
    try:
        value = float(value)
    except (TypeError, ValueError):
        return None

    if not math.isfinite(value):
        return None

    return value % 360.0


def classify_encounter(
    own_heading_deg,
    target_bearing_deg,
    target_heading_deg,
    target_cog_deg,
    own_speed_mps,
    target_speed_mps,
    head_on_bearing_deg=10.0,
    reciprocal_tolerance_deg=20.0,
    speed_margin_mps=0.2,
):
    """
    Simplified COLREG geometry classifier.

    This is advisory logic, not a complete COLREG
    compliance implementation.
    """

    own_heading = valid_angle(
        own_heading_deg
    )

    bearing = valid_angle(
        target_bearing_deg
    )

    target_heading = valid_angle(
        target_heading_deg
    )

    target_cog = valid_angle(
        target_cog_deg
    )

    # Prefer heading for COLREG vessel aspect.
    # Fall back to COG if heading is unavailable.
    target_direction = (
        target_heading
        if target_heading is not None
        else target_cog
    )

    if (
        own_heading is None
        or bearing is None
        or target_direction is None
    ):
        return {
            'encounter': 'UNKNOWN',
            'role': 'UNKNOWN',
            'action': 'MONITOR',
        }

    # Target bearing relative to own bow:
    # + = starboard, - = port.
    rel_bearing = wrap_180(
        bearing - own_heading
    )

    # Is target approximately reciprocal to us?
    reciprocal_error = abs(
        wrap_180(
            target_direction
            - (own_heading + 180.0)
        )
    )

    # Where does the target see us?
    bearing_target_to_own = (
        bearing + 180.0
    ) % 360.0

    own_as_seen_from_target = wrap_180(
        bearing_target_to_own
        - target_direction
    )

    # --------------------------------------------------
    # HEAD-ON
    # --------------------------------------------------
    if (
        abs(rel_bearing)
        <= head_on_bearing_deg
        and reciprocal_error
        <= reciprocal_tolerance_deg
    ):
        return {
            'encounter': 'HEAD_ON',
            'role': 'BOTH_GIVE_WAY',
            'action': 'ALTER_STARBOARD',
        }

    # --------------------------------------------------
    # OVERTAKING
    #
    # Ownship lies in target vessel's stern sector:
    # more than 22.5 deg abaft either beam.
    # That corresponds to |relative angle| > 112.5 deg.
    # --------------------------------------------------
    if (
        abs(own_as_seen_from_target)
        > 112.5
        and own_speed_mps is not None
        and target_speed_mps is not None
        and own_speed_mps
        > target_speed_mps + speed_margin_mps
    ):
        return {
            'encounter': 'OVERTAKING',
            'role': 'GIVE_WAY',
            'action': 'KEEP_CLEAR',
        }

    # Target is overtaking us.
    if (
        abs(rel_bearing) > 112.5
        and own_speed_mps is not None
        and target_speed_mps is not None
        and target_speed_mps
        > own_speed_mps + speed_margin_mps
    ):
        return {
            'encounter': 'BEING_OVERTAKEN',
            'role': 'STAND_ON',
            'action': 'MAINTAIN_COURSE_SPEED',
        }

    # --------------------------------------------------
    # CROSSING
    # --------------------------------------------------
    if (
        head_on_bearing_deg
        < rel_bearing
        < 112.5
    ):
        return {
            'encounter': 'CROSSING_STARBOARD',
            'role': 'GIVE_WAY',
            'action': 'KEEP_CLEAR',
        }

    if (
        -112.5
        < rel_bearing
        < -head_on_bearing_deg
    ):
        return {
            'encounter': 'CROSSING_PORT',
            'role': 'STAND_ON',
            'action': 'MAINTAIN_COURSE_SPEED',
        }

    return {
        'encounter': 'OTHER',
        'role': 'MONITOR',
        'action': 'MONITOR',
    }
