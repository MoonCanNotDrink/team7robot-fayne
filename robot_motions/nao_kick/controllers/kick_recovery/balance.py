"""Closed-loop balance helpers for the NAO kick controller.

Balancer is an ankle/hip strategy controller. It is passed to
MotionPlayer.play(..., feedback=balancer) and every simulation step it
returns small joint offsets that are added on top of the keyframe pose,
pushing the torso back to the tilt it had when the balancer was created.

Sign conventions (derived from the NAO joint axes, all roll joints turn
about +x, all pitch joints about +y):
  pitch lean = z of the torso's forward axis: > 0 leaning back
  roll lean  = z of the torso's left axis:    > 0 leaning right
  AnklePitch more negative tilts the shin (and body) forwards
  AnkleRoll  more positive tilts the shin (and body) to the left
So leaning back is corrected with -k * pitch on AnklePitch, and leaning
right with +k * roll on AnkleRoll.
"""
import math

# Default gains. Lean is roughly the tilt angle in rad for small angles.
KP_PITCH = 0.6      # TUNE rad of ankle pitch per rad of forward/back lean
KD_PITCH = 0.06     # TUNE damping on the lean rate
KP_ROLL = 0.6       # TUNE rad of ankle roll per rad of sideways lean
KD_ROLL = 0.05      # TUNE
KP_HIP = 0.3        # TUNE hip pitch share of the forward/back correction
LIMIT = 0.15        # largest offset sent to any joint (rad)
DEADBAND = 0.01     # ignore lean smaller than this (rad), so standing still is quiet
RATE_FILTER = 0.3   # low-pass weight of the newest lean-rate sample (0..1)


def _clamp(x, lim):
    return max(-lim, min(lim, x))


class Balancer:
    """Callable that returns {joint: offset} each step.

    legs: which legs carry weight, "LR" for both, "L" or "R" for one foot.
    roll: also correct sideways lean (off for recorded walking motions,
          which shift the weight sideways on purpose).
    """

    def __init__(self, nao, legs="LR", roll=True, gain=1.0):
        self.nao = nao
        self.legs = legs
        self.roll = roll
        self.gain = gain
        self.dt = nao.ts / 1000.0
        fz, lz, _ = nao.torso_axes()
        self.base_p, self.base_r = fz, lz        # treat the current tilt as upright
        self.prev_p, self.prev_r = fz, lz
        self.rate_p = self.rate_r = 0.0

    def tilt(self):
        """(pitch lean, roll lean) relative to the starting posture, in ~rad."""
        fz, lz, _ = self.nao.torso_axes()
        return fz - self.base_p, lz - self.base_r

    def __call__(self):
        fz, lz, _ = self.nao.torso_axes()
        p, r = fz - self.base_p, lz - self.base_r
        a = RATE_FILTER
        self.rate_p = (1 - a) * self.rate_p + a * (fz - self.prev_p) / self.dt
        self.rate_r = (1 - a) * self.rate_r + a * (lz - self.prev_r) / self.dt
        self.prev_p, self.prev_r = fz, lz
        if abs(p) < DEADBAND:
            p = 0.0
        if abs(r) < DEADBAND:
            r = 0.0
        g = self.gain
        ank_p = _clamp(-g * (KP_PITCH * p + KD_PITCH * self.rate_p), LIMIT)
        hip_p = _clamp(-g * KP_HIP * p, LIMIT)
        ank_r = _clamp(g * (KP_ROLL * r + KD_ROLL * self.rate_r), LIMIT) if self.roll else 0.0
        offs = {}
        for leg in self.legs:
            offs[leg + "AnklePitch"] = ank_p
            offs[leg + "HipPitch"] = hip_p
            if self.roll:
                offs[leg + "AnkleRoll"] = ank_r
        return offs


class FootSensors:
    """Load on each foot from the NAO's LFsr / RFsr force sensors (if present)."""

    def __init__(self, robot, ts):
        self.fsr = {}
        for side in "LR":
            try:
                dev = robot.getDevice(side + "Fsr")
            except Exception:
                dev = None
            if dev is not None:
                dev.enable(ts)
                self.fsr[side] = dev
        self.available = len(self.fsr) == 2

    def loads(self):
        """(left N, right N) or None when the sensors are missing or not ready."""
        if not self.available:
            return None
        out = []
        for side in "LR":
            v = self.fsr[side].getValues()
            if v is None or any(x != x for x in v):
                return None
            out.append(math.sqrt(sum(x * x for x in v)))    # force magnitude, axis-independent
        return tuple(out)

    def share(self, side):
        """Fraction (0..1) of the total load on the given foot, or None."""
        ld = self.loads()
        if ld is None or ld[0] + ld[1] < 1.0:
            return None
        return (ld[0] if side == "L" else ld[1]) / (ld[0] + ld[1])
