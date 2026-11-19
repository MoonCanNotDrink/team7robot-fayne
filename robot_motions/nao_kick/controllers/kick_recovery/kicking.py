"""Kick execution and ball approach behavior for the NAO controller."""

import math
import sys

import poses
from balance import Balancer
from motion_player import motion_to_sequence
from settings import *  # shared kick and positioning tuning values


class KickingMixin:
    def wait_until_still(self, max_s=STILL_MAX_S):
        """Stand with balance control until the body stops swaying (or max_s).
        Returns False if the robot fell."""
        bal = Balancer(self)
        bal.base_p = bal.base_r = 0.0          # aim for an upright torso, not the current sway
        still = [0]

        def check():
            w = self.me.getVelocity()[3:6]
            still[0] = still[0] + 1 if math.sqrt(sum(x * x for x in w)) < STILL_RATE else 0
            return self.fall_check() or still[0] * self.ts / 1000.0 >= STILL_HOLD_S

        self._tilt_steps = 0
        self.player.play([(max_s, {})], check=check, feedback=bal)
        return not self.is_fallen()

    def kick_profile(self, kind=None, foot=KICK_FOOT):
        """Settings for a kick type, with the direction mirrored for the left foot
        and turned into radians. Unknown types fall back to 'normal'."""
        kind = kind or self.kick_type
        if kind not in KICK_TYPES:
            print(f"Unknown kick type '{kind}', using 'normal'")
            kind = "normal"
        prof = dict(KICK_TYPES[kind], kind=kind)
        sign = 1.0 if foot == "R" else -1.0
        prof["direction"] = math.radians(prof["direction"]) * sign
        prof["side"] = -1.0 if foot == "R" else 1.0     # kicking foot's side in the robot's y
        return prof

    def kick(self, strength=None, foot=KICK_FOOT, kind=None):
        """Kick and return (peak ball speed m/s, fell during kick).

        kind is a key of KICK_TYPES ('slow', 'normal', 'fast', 'side'); strength
        overrides the type's own strength (kick_test uses this to sweep it).

        The custom kick runs in closed loop:
          1. shift the weight over the support foot (balance on both feet)
          2. with foot sensors, keep shifting until the kicking foot is unloaded
          3. swing on one foot with ankle balance on the support foot; if the
             torso tilts past KICK_ABORT_TILT, plant the foot instead of falling
          4. move back between the feet
        """
        peak = [0.0]

        def watch():
            peak[0] = max(peak[0], self.ball_speed())
            return self.fall_check()

        self._tilt_steps = 0
        if USE_BUILTIN_SHOOT and "Shoot" in self.motions:
            res = self.player.play(motion_to_sequence(self.motions["Shoot"]), check=watch)
            fell = res == "aborted" or self.is_fallen()
            if fell:
                self._report_kick_fall("Shoot.motion", self.player.frame_index)
                self.brace_for_fall()
            return peak[0], fell

        prof = self.kick_profile(kind, foot)
        if strength is None:
            strength = prof["strength"]
        build = poses.side_kick_phases if prof["kind"] == "side" else poses.kick_phases
        print(f"  {prof['kind']} kick, {foot} foot, strength {strength:.2f}")
        support = "L" if foot == "R" else "R"
        stages = dict(build(foot, strength, tempo=prof["tempo"]))
        both = Balancer(self)                          # baseline: the settled standing tilt

        # 1. weight shift
        res = self.player.play(stages["shift"], check=watch, feedback=both)
        if res != "done":
            return self._kick_fell("shift", peak[0])

        # 2. make sure the kicking foot is (nearly) unloaded before lifting it,
        #    then rebuild the swing for the shift actually used
        shift = self._unload(foot, both, watch)
        if shift is None:
            return self._kick_fell("shift", peak[0])
        stages = dict(build(foot, strength, shift=shift, tempo=prof["tempo"]))

        # 3. swing on one foot
        one = Balancer(self, legs=support)
        one.base_p, one.base_r = both.base_p, both.base_r
        tipping = [False]

        def watch_swing():
            if watch():
                return True
            p, r = one.tilt()
            if math.hypot(p, r) > KICK_ABORT_TILT:
                tipping[0] = True
                return True
            return False

        res = self.player.play(stages["swing"], check=watch_swing, feedback=one)
        if tipping[0] and not self.is_fallen():
            p, r = one.tilt()
            print(f"  kick aborted: tilt pitch {p:.2f}, roll {r:.2f} rad - planting the foot")
            self.player.play(poses.plant_foot_sequence(foot), feedback=Balancer(self))
            self.stand(0.6)
            self._tilt_steps = 0
            return peak[0], self.is_fallen()
        if res != "done":
            return self._kick_fell("swing", peak[0])

        # 4. back to the centre
        res = self.player.play(stages["centre"], check=watch, feedback=Balancer(self))
        if res != "done" or self.is_fallen():
            return self._kick_fell("centre", peak[0])
        return peak[0], False

    def _unload(self, foot, bal, watch):
        """Grow the weight shift until the kicking foot carries < UNLOAD_SHARE.
        Returns the weight shift used (rad), or None if the robot fell.
        Without foot sensors the shift stays at WEIGHT_SHIFT."""
        shift = poses.WEIGHT_SHIFT
        share = self.feet.share(foot)
        if share is None:
            return shift
        t_end = self.robot.getTime() + UNLOAD_MAX_S
        while share is not None and share > UNLOAD_SHARE:
            if shift >= poses.WEIGHT_SHIFT_MAX or self.robot.getTime() > t_end:
                print(f"  weight shift {shift:.2f} rad: kicking foot still carries {share:.0%}")
                break
            shift = min(poses.WEIGHT_SHIFT_MAX, shift + 0.02)
            res = self.player.play([(0.1, poses.weight_shift_pose(foot, shift)), (0.1, {})],
                                   check=watch, feedback=bal)
            if res != "done":
                return None
            share = self.feet.share(foot)
        return shift

    def _kick_fell(self, stage, peak):
        self._report_kick_fall(stage, self.player.frame_index, KICK_STAGE_PHASES.get(stage, []))
        if self.is_fallen() or self._tilt_steps:
            self.brace_for_fall()
        return peak, True

    def _report_kick_fall(self, stage, i, names=()):
        phase = names[i] if i < len(names) else f"frame {i}"
        fz, lz, uz = self.torso_axes()
        print(f"  fell during kick stage '{stage}', phase '{phase}' "
              f"(torso forward_z={fz:.2f}, left_z={lz:.2f}): "
              + ("fell forwards" if fz < -0.3 else "fell backwards" if fz > 0.3 else
                 "fell to the left" if lz < -0.3 else "fell to the right" if lz > 0.3 else "tilted"))

    def stance_target(self, target, foot, kind=None):
        """Where the robot should stand, and which way it should face, to kick
        the ball towards target with the given kick type.

        Returns (gx, gy, kick_dir, face, (bx, by, ux, uy, lx, ly)): stance point,
        ball direction, robot heading, and the ball position with the unit
        vectors along and to the left of the kick direction.
        """
        prof = self.kick_profile(kind, foot)
        bx, by = self.ball_pos()
        kick_dir = math.atan2(target[1] - by, target[0] - bx)
        face = wrap(kick_dir - prof["direction"])        # side kick: stand side-on to the target
        ux, uy = math.cos(kick_dir), math.sin(kick_dir)
        lx, ly = -uy, ux                                # left of the kick direction
        fx, fy = prof["ball_x"], prof["side"] * prof["ball_y"]   # ball in the robot frame
        c, s = math.cos(face), math.sin(face)
        gx = bx - (c * fx - s * fy)
        gy = by - (s * fx + c * fy)
        return gx, gy, kick_dir, face, (bx, by, ux, uy, lx, ly)

    def errors_to(self, gx, gy):
        """Position error to (gx, gy) in the robot frame: (forward, left, distance)."""
        x, y, yaw = self.robot_pose()
        ex_w, ey_w = gx - x, gy - y
        c, s = math.cos(yaw), math.sin(yaw)
        return c * ex_w + s * ey_w, -s * ex_w + c * ey_w, math.hypot(ex_w, ey_w)

    def ready_to_kick(self, target=None, foot=KICK_FOOT, kind=None):
        """Return True when the ball is already at the kicking foot and aligned."""
        target = target or self.target
        gx, gy, kick_dir, face, (bx, by, ux, uy, _, _) = self.stance_target(target, foot, kind)
        ex, ey, _ = self.errors_to(gx, gy)
        x, y, _ = self.robot_pose()
        # The ball must be ahead of the robot; this avoids treating a ball
        # beside or behind the kicking foot as ready just because the target
        # stance point happens to be nearby.
        ball_forward = (bx - x) * ux + (by - y) * uy
        heading_error = abs(wrap(face - self.robot_pose()[2]))
        return (ball_forward > 0.0
                and abs(ex) <= POS_TOL_X
                and abs(ey) <= POS_TOL_Y
                and heading_error <= HEAD_TOL)

    def position_for_kick(self, target=None, foot=KICK_FOOT, max_actions=30, kind=None):
        """Walk until the ball is at the kicking foot, facing the way the kick type needs
        (towards the target, or side-on for a side kick).

        Returns True when lined up, False if it fell or ran out of actions.
        """
        target = target or self.target
        for _ in range(max_actions):
            gx, gy, kick_dir, face, (bx, by, ux, uy, lx, ly) = self.stance_target(target, foot, kind)
            x, y, yaw = self.robot_pose()
            ahead = (x - bx) * ux + (y - by) * uy > -0.10    # level with or past the ball
            dist = math.hypot(gx - x, gy - y)
            ax_, ay_ = bx - ux * 0.45, by - uy * 0.45           # approach point behind the ball
            dist_a = math.hypot(ax_ - x, ay_ - y)

            if ahead:
                # go round the side of the ball, never through it
                s = 1.0 if (x - bx) * lx + (y - by) * ly >= 0 else -1.0
                res = self.walk_to(bx - ux * 0.25 + s * lx * 0.45, by - uy * 0.25 + s * ly * 0.45, tol=0.12)
            elif dist > 0.35 and dist_a > 0.12:
                # far: walk to the approach point 0.45 m behind the ball, on the kick line
                res = self.walk_to(ax_, ay_, tol=0.12)
            else:
                # close: turn to the kicking heading, then fix sideways and forward errors
                ex, ey, _ = self.errors_to(gx, gy)
                if abs(wrap(face - yaw)) > HEAD_TOL:
                    res = self.turn_to(face)
                elif abs(ey) > POS_TOL_Y:
                    sgn = 1.0 if ey > 0 else -1.0
                    name = "SideStepLeft" if sgn > 0 else "SideStepRight"
                    if name not in self.motions:
                        print("Motion not available:", name)
                        return False
                    # Finish the complete two-leg motion. Interrupting on the
                    # first position crossing can stop before the other leg steps.
                    seq = motion_to_sequence(self.motions[name],
                                             time_scale=1.0 / SIDE_STEP_SPEED)
                    for _, pose in seq:
                        # scale ankle roll with hip roll so the sole stays flat
                        for joint in ("LHipRoll", "RHipRoll", "LAnkleRoll", "RAnkleRoll"):
                            if joint in pose:
                                pose[joint] *= SIDE_STEP_WIDTH
                    self._tilt_steps = 0
                    res = self.player.play(seq, check=self.fall_check)
                    if res == "quit":
                        sys.exit(0)
                    if res != "done" or self.is_fallen():
                        if res == "aborted":
                            self.brace_for_fall()
                        return False
                    print(f"  {name} complete; checking lateral position again")
                elif ex > POS_TOL_X:
                    res = self.play_until("Forwards", lambda: self.errors_to(gx, gy)[0] < POS_TOL_X * 0.5,
                                          max_time=8.0)
                elif ex < -POS_TOL_X:
                    res = self.play_until("Backwards", lambda: self.errors_to(gx, gy)[0] > -POS_TOL_X * 0.5,
                                          max_time=8.0)
                else:
                    self.stand(0.6)
                    return True
            if res == "fallen" or self.is_fallen():
                return False
        print("Could not line up within the action limit.")
        return False

