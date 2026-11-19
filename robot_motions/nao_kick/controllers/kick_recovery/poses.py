"""Joint poses and keyframe sequences for the NAO V5 in Webots.

Angles are in radians, using the NAO joint conventions:
  HipPitch < 0 swings the leg forwards, KneePitch > 0 bends the knee,
  LHipRoll > 0 / RHipRoll < 0 move a leg outwards,
  LShoulderRoll > 0 / RShoulderRoll < 0 move an arm outwards.

A keyframe sequence is a list of (duration_s, {joint: angle}). Joints that a
frame does not mention keep their previous target.

Every number marked TUNE is a starting value. Check it in Webots and adjust;
the Webots robot window (double-click the robot) shows live joint angles.
"""

JOINTS = [
    "HeadYaw", "HeadPitch",
    "LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll", "LWristYaw",
    "RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll", "RWristYaw",
    "LHipYawPitch", "LHipRoll", "LHipPitch", "LKneePitch", "LAnklePitch", "LAnkleRoll",
    "RHipYawPitch", "RHipRoll", "RHipPitch", "RKneePitch", "RAnklePitch", "RAnkleRoll",
]

# Joints whose sign flips when a pose is mirrored left <-> right
_MIRROR_NEGATE = ("HeadYaw", "ShoulderRoll", "ElbowYaw", "ElbowRoll",
                  "WristYaw", "HipRoll", "AnkleRoll")

STAND = {
    "HeadYaw": 0.0, "HeadPitch": 0.0,
    "LShoulderPitch": 1.45, "LShoulderRoll": 0.20, "LElbowYaw": -1.20, "LElbowRoll": -0.50, "LWristYaw": 0.0,
    "RShoulderPitch": 1.45, "RShoulderRoll": -0.20, "RElbowYaw": 1.20, "RElbowRoll": 0.50, "RWristYaw": 0.0,
    "LHipYawPitch": 0.0, "LHipRoll": 0.0, "LHipPitch": -0.45, "LKneePitch": 0.90, "LAnklePitch": -0.45, "LAnkleRoll": 0.0,
    "RHipYawPitch": 0.0, "RHipRoll": 0.0, "RHipPitch": -0.45, "RKneePitch": 0.90, "RAnklePitch": -0.45, "RAnkleRoll": 0.0,
}

# Relaxed pose used while lying down (legs straight, arms by the sides)
LYING = dict(STAND, **{
    "LHipPitch": 0.0, "LKneePitch": 0.0, "LAnklePitch": 0.0,
    "RHipPitch": 0.0, "RKneePitch": 0.0, "RAnklePitch": 0.0,
    "LShoulderPitch": 1.5, "RShoulderPitch": 1.5,
})


def mirror(pose):
    """Swap left and right in a pose (used to build left-foot kicks etc.)."""
    out = {}
    for name, val in pose.items():
        if name[0] == "L":
            new = "R" + name[1:]
        elif name[0] == "R":
            new = "L" + name[1:]
        else:
            new = name
        if any(name.endswith(j) for j in _MIRROR_NEGATE):
            val = -val
        out[new] = val
    return out


def mirror_sequence(seq):
    return [(d, mirror(p)) for d, p in seq]


def _clamp01(x):
    return max(0.0, min(1.0, x))


# --------------------------------------------------------------------------
# Kick
# --------------------------------------------------------------------------
WEIGHT_SHIFT = 0.22         # TUNE hip/ankle roll used to move the body over the support foot
                            #      (with foot sensors this is only the starting value, see below)
WEIGHT_SHIFT_MAX = 0.32     # with foot sensors the shift grows up to this until the foot is unloaded
ARM_SWING = 0.5             # TUNE arm counter-swing during the kick (0 = arms still)


def weight_shift_pose(foot="R", shift=WEIGHT_SHIFT):
    """Parallel hip/ankle roll that moves the body over the support foot while
    keeping the torso upright and both soles flat."""
    sh = -shift if foot == "R" else shift   # right-foot kick: lean over the LEFT foot
    return {"LHipRoll": sh, "RHipRoll": sh, "LAnkleRoll": -sh, "RAnkleRoll": -sh}


def kick_phases(foot="R", strength=0.7, shift=WEIGHT_SHIFT, tempo=1.0):
    """The kick as named stages, so the controller can check balance between them.

    Returns a list of (stage name, keyframe sequence):
      shift   stand, then move the body over the support foot (both feet down)
      swing   lift, back-swing, swing through, retract, put the foot down
      centre  move the body back between the feet
    The support-side arm opens a little for sideways balance and the arms
    counter-swing against the kicking leg to cancel the yaw it causes.
    tempo scales only the back-swing and swing-through (< 1 faster, > 1 slower),
    so a slow kick does not spend longer standing on one foot.
    strength in [0, 1] changes how far back the leg swings and how fast it
    swings through; both raise the ball speed.
    """
    s = _clamp01(strength)
    arm = ARM_SWING * (0.5 + 0.5 * s)
    shift_seq = [
        (0.6, dict(STAND)),
        (0.9, dict(weight_shift_pose("R", shift), LShoulderRoll=0.40)),
        (0.3, {}),
    ]
    swing_seq = [
        # lift the kicking foot
        (0.6, {"RHipPitch": -0.85, "RKneePitch": 1.70, "RAnklePitch": -0.85}),
        # back-swing (bigger for stronger kicks); arms wind up the other way    TUNE
        (0.35 * tempo, {"RHipPitch": -0.50 + 0.30 * s, "RKneePitch": 1.80, "RAnklePitch": -0.90,
                "LShoulderPitch": 1.45 + 0.5 * arm, "RShoulderPitch": 1.45 - 0.5 * arm}),
        # swing through: faster and further for stronger kicks; arms counter-swing  TUNE
        ((0.32 - 0.18 * s) * tempo, {"RHipPitch": -1.00 - 0.25 * s, "RKneePitch": 0.35, "RAnklePitch": 0.10,
                           "LShoulderPitch": 1.45 - arm, "RShoulderPitch": 1.45 + 0.5 * arm}),
        (0.15, {}),
        # retract and put the foot down, still leaning on the support foot
        (0.5, {"RHipPitch": -0.85, "RKneePitch": 1.70, "RAnklePitch": -0.85,
               "LShoulderPitch": 1.45, "RShoulderPitch": 1.45}),
        (0.6, {"RHipPitch": -0.45, "RKneePitch": 0.90, "RAnklePitch": -0.45}),
        (0.4, {}),
    ]
    centre_seq = [
        (0.9, {"LHipRoll": 0.0, "RHipRoll": 0.0, "LAnkleRoll": 0.0, "RAnkleRoll": 0.0,
               "LShoulderRoll": STAND["LShoulderRoll"]}),
        (0.4, {}),
    ]
    stages = [("shift", shift_seq), ("swing", swing_seq), ("centre", centre_seq)]
    if foot == "R":
        return stages
    return [(name, mirror_sequence(seq)) for name, seq in stages]


SIDE_KICK_LIFT = 0.08       # TUNE how high the foot is lifted for the side kick (as hip pitch, rad)


def side_kick_phases(foot="R", strength=0.7, shift=WEIGHT_SHIFT, tempo=1.0):
    """Side kick: the ball sits beside the OUTSIDE of the kicking foot and the
    leg swings outwards (hip roll), pushing the ball sideways, away from the
    support leg. A right-foot side kick sends the ball to the robot's right.

    Same stages as kick_phases so the controller balances it the same way.
    The sole is kept parallel to the floor by setting AnkleRoll = -HipRoll.
    """
    s = _clamp01(strength)
    lift = -0.45 - SIDE_KICK_LIFT                # hip pitch a little more forward than STAND
    # Rolls are relative to the weight-shift pose (RHipRoll = -shift there). There is
    # no inward back-swing: after the shift the feet are only ~10 cm apart, so drawing
    # the foot in would hit the support leg. Power comes from the swing speed instead.
    rest = -shift
    out = rest - (0.20 + 0.25 * s)               # swing out (RHipRoll more negative)      TUNE
    shift_seq = [
        (0.6, dict(STAND)),
        (0.9, dict(weight_shift_pose("R", shift), LShoulderRoll=0.40)),
        (0.3, {}),
    ]
    # the support leg keeps the weight-shift roll; the kicking leg gets its own roll on top
    swing_seq = [
        # lift the kicking foot just clear of the ground
        (0.5, {"RHipPitch": lift, "RKneePitch": 1.10, "RAnklePitch": -(1.10 + lift)}),  # sole flat
        # swing out: faster for stronger kicks; the left arm opens to balance
        ((0.30 - 0.18 * s) * tempo, {"RHipRoll": out, "RAnkleRoll": -out, "LShoulderRoll": 0.60}),
        (0.15, {}),
        # bring the leg back under the hip and put the foot down
        (0.5, {"RHipRoll": rest, "RAnkleRoll": -rest, "LShoulderRoll": 0.40}),
        (0.6, {"RHipPitch": -0.45, "RKneePitch": 0.90, "RAnklePitch": -0.45}),
        (0.4, {}),
    ]
    centre_seq = [
        (0.9, {"LHipRoll": 0.0, "RHipRoll": 0.0, "LAnkleRoll": 0.0, "RAnkleRoll": 0.0,
               "LShoulderRoll": STAND["LShoulderRoll"]}),
        (0.4, {}),
    ]
    stages = [("shift", shift_seq), ("swing", swing_seq), ("centre", centre_seq)]
    if foot == "R":
        return stages
    return [(name, mirror_sequence(seq)) for name, seq in stages]


def kick_sequence(foot="R", strength=0.7):
    """The whole kick as one open-loop keyframe sequence (no balance checks)."""
    return [frame for _, seq in kick_phases(foot, strength) for frame in seq]


def plant_foot_sequence(foot="R"):
    """Emergency: put the kicking foot straight down and widen the stance a bit."""
    seq = [
        (0.18, {"RHipPitch": -0.45, "RKneePitch": 0.90, "RAnklePitch": -0.45,
                "RHipRoll": -0.05, "RAnkleRoll": 0.05,
                "LShoulderPitch": 1.45, "RShoulderPitch": 1.45}),
        (0.5, {"LHipRoll": 0.0, "LAnkleRoll": 0.0}),
    ]
    return seq if foot == "R" else mirror_sequence(seq)


def brace_pose(direction):
    """Protective pose once a fall cannot be stopped.

    forward: arms out in front to take the impact, head back.
    backward / side: chin tucked, arms forward, knees bent to drop the hips
    so the robot sits down instead of hitting the floor with its head.
    """
    if direction == "forward":
        return {"HeadPitch": -0.5, "LShoulderPitch": 0.2, "RShoulderPitch": 0.2,
                "LShoulderRoll": 0.25, "RShoulderRoll": -0.25,
                "LElbowRoll": -0.3, "RElbowRoll": 0.3}
    return {"HeadPitch": 0.45, "LShoulderPitch": 0.6, "RShoulderPitch": 0.6,
            "LShoulderRoll": 0.15, "RShoulderRoll": -0.15,
            "LHipPitch": -0.9, "RHipPitch": -0.9, "LKneePitch": 1.8, "RKneePitch": 1.8,
            "LAnklePitch": -0.9, "RAnklePitch": -0.9}


# --------------------------------------------------------------------------
# Get-up from the back: roll onto the front, then stand up from the front
# --------------------------------------------------------------------------
def rollover_sequence(side="L"):
    """Roll from lying on the back onto the front, towards the given side.

    This is a first design to tune in Webots: arms go overhead, the far leg
    bends and swings across, and the far arm sweeps over the body.
    """
    seq = [
        (0.5, dict(LYING)),
        # arms overhead, elbows straight                                  TUNE
        (0.6, {"LShoulderPitch": -1.2, "RShoulderPitch": -1.2,
               "LShoulderRoll": 0.3, "RShoulderRoll": -0.3,
               "LElbowRoll": -0.05, "RElbowRoll": 0.05}),
        # bend the right leg and turn it towards the left                 TUNE
        (0.5, {"RHipPitch": -1.2, "RKneePitch": 1.6, "RHipRoll": 0.3,
               "LHipYawPitch": -0.6, "RHipYawPitch": -0.6}),
        # swing the right leg and right arm across, open the left arm     TUNE
        (0.4, {"RHipPitch": -1.5, "LShoulderRoll": 1.2,
               "RShoulderPitch": 0.5, "RShoulderRoll": 0.3, "HeadYaw": 1.0}),
        (0.8, {}),                              # let gravity finish the roll
        # straighten out lying on the front, arms ready to push
        (0.6, {"LHipYawPitch": 0.0, "RHipYawPitch": 0.0, "RHipRoll": 0.0,
               "RHipPitch": 0.0, "RKneePitch": 0.0, "HeadYaw": 0.0,
               "LShoulderPitch": 1.0, "RShoulderPitch": 1.0,
               "LShoulderRoll": 0.3, "RShoulderRoll": -0.3}),
        (0.4, {}),
    ]
    return seq if side == "L" else mirror_sequence(seq)
