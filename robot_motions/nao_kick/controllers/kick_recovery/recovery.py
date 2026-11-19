"""Fall detection and get-up behavior for the NAO controller."""

import math
import sys

import poses
from motion_player import motion_to_sequence
from settings import *  # shared motion and balance tuning values


class RecoveryMixin:
    def torso_axes(self):
        """World z-components of the torso's forward, left and up axes."""
        R = self.me.getOrientation()          # 3x3 row-major
        return R[6], R[7], R[8]

    def acc_values(self):
        v = self.acc.getValues()
        return [0.0, 0.0, 9.81] if any(x != x for x in v) else v

    def is_fallen(self):
        """Instant check: torso tilted past UPRIGHT_MIN."""
        return self.torso_axes()[2] < UPRIGHT_MIN

    def fall_check(self):
        """Debounced check for use during motions (tilt must persist).

        Triggers when the torso is past UPRIGHT_MIN, or earlier when it is past
        FALLING_UZ and still tipping, so there is time to brace for the impact.
        """
        uz = self.torso_axes()[2]
        t = self.robot.getTime()
        prev_uz, prev_t = self._prev_uz
        rate = (uz - prev_uz) / (t - prev_t) if t > prev_t else 0.0
        self._prev_uz = (uz, t)
        if uz < UPRIGHT_MIN or (uz < FALLING_UZ and rate < FALLING_RATE):
            self._tilt_steps += 1
        else:
            self._tilt_steps = 0
        return self._tilt_steps * self.ts / 1000.0 >= FALL_CONFIRM_S

    def brace_for_fall(self):
        """A fall cannot be stopped any more: take a protective pose quickly."""
        fz = self.torso_axes()[0]
        direction = "forward" if fz < -0.3 else "backward"
        print(f"  falling {direction}: bracing")
        self.player.play([(0.2, poses.brace_pose(direction))])

    def lying_state(self):
        """'up', 'back', 'front' or 'side' from the torso orientation."""
        fz, lz, uz = self.torso_axes()
        if uz > 0.7:
            return "up"
        # A robot tilted between its back and side can have a fairly large fz.
        # Classify by the dominant horizontal torso axis so side falls take the
        # side-to-front recovery path instead of starting GetUpBack directly.
        if abs(lz) > abs(fz):
            return "side"
        if fz > 0.5:
            return "back"          # chest facing the sky
        if fz < -0.5:
            return "front"         # chest facing the floor
        return "side"

    def balanced_stand(self, settle=SETTLE_S, duration=TO_STAND_S):
        """Settle after a get-up motion, then blend to STAND with lean control.

        Lean is the z-component of the torso's forward axis: > 0 leaning back,
        < 0 leaning forward. Leaning back is corrected by tilting the shins
        (AnklePitch more negative) and the torso (HipPitch more negative)
        forwards, and the other way round. Returns True if still upright.
        """
        dt = self.ts / 1000.0
        start = dict(self.player.target)
        end = dict(start)
        for name, val in poses.STAND.items():
            if name in self.motors:
                end[name] = self.player._clamp(name, val)
        n_settle = int(settle / dt)
        n_blend = max(1, int(duration / dt))
        prev = self.torso_axes()[0]
        tilt_steps = 0
        for i in range(n_settle + n_blend):
            a = 0.0 if i < n_settle else 0.5 - 0.5 * math.cos(math.pi * (i - n_settle + 1) / n_blend)
            lean = self.torso_axes()[0]
            rate = (lean - prev) / dt
            prev = lean
            ank = max(-BAL_MAX, min(BAL_MAX, -(BAL_KP_ANKLE * lean + BAL_KD_ANKLE * rate)))
            hip = max(-BAL_MAX, min(BAL_MAX, -BAL_KP_HIP * lean))
            for name in end:
                v = start[name] + a * (end[name] - start[name])
                if name.endswith("AnklePitch"):
                    v += ank
                elif name.endswith("HipPitch"):
                    v += hip
                v = self.player._clamp(name, v)
                if v == v:
                    self.motors[name].setPosition(v)
            if self.robot.step(self.ts) == -1:
                sys.exit(0)
            tilt_steps = tilt_steps + 1 if self.is_fallen() else 0
            if tilt_steps * dt > 0.3:
                self.player.target = self.player.current_pose()
                return False
        self.player.target = end
        return True

    def recover(self):
        """Get up from any fall. Returns (success, seconds taken, attempts).

        back  -> GetUpBack.motion (Cyberbotics wrestling)
        front -> GetUpFront.motion, else Webots' StandUpFromFront.motion
        side  -> GetUpSide.motion, roll onto the front, then GetUpFront.motion
        """
        t0 = self.robot.getTime()
        self.step(0.5)                                   # let the robot settle first
        if self.lying_state() == "up":                   # not actually down: do nothing
            self.stand(0.5)
            return True, self.robot.getTime() - t0, 0
        roll_side = "L"
        for attempt in range(1, MAX_GETUP_ATTEMPTS + 1):
            state = self.lying_state()
            side_fall = state == "side"
            fz, lz, uz = self.torso_axes()
            print(f"  get-up attempt {attempt}: lying on {state} "
                  f"(torso forward_z={fz:.2f}, left_z={lz:.2f}, up_z={uz:.2f})")
            if state == "up":
                return True, self.robot.getTime() - t0, attempt - 1
            if state == "side":
                fz, lz, uz = self.torso_axes()
                if "GetUpSide" in self.motions:
                    # The authored clip is for the robot lying on its right side;
                    # mirror it when the torso's left axis points down instead.
                    print("  playing GetUpSide.motion")
                    seq = motion_to_sequence(self.motions["GetUpSide"])
                    if lz < 0:
                        seq = poses.mirror_sequence(seq)
                    self.player.play(seq)
                else:
                    print("  GetUpSide.motion unavailable; using shoulder push")
                    pose = {"RShoulderRoll": -1.2} if lz > 0 else {"LShoulderRoll": 1.2}
                    self.player.play([(0.4, pose), (1.0, {})])
                self.step(0.3)
                state = self.lying_state()
                print(f"  side motion ended on {state}")
                if state == "side":
                    print("  side roll did not reach the back or front; retrying")
                    continue
            if state == "back":
                if side_fall:
                    # Side recovery must reach the front before standing. Do not
                    # start the back get-up if the side roll lands supine.
                    print("  side motion reached back, not front; retrying side-to-front roll")
                    continue
                elif "GetUpBack" in self.motions:
                    self.play_motion("GetUpBack", abort_on_fall=False)
                else:
                    self.player.play(poses.rollover_sequence(roll_side))
                    self.step(0.5)
                    roll_side = "R" if roll_side == "L" else "L"
                    state = self.lying_state()
            if state == "front":
                name = "GetUpFront" if "GetUpFront" in self.motions else "StandUpFromFront"
                if side_fall:
                    print("  playing front get-up after side roll")
                self.play_motion(name, abort_on_fall=False)
            # the critical moment: straight after the get-up the robot is upright but
            # still moving, so settle and balance instead of jumping to the stand pose
            if self.torso_axes()[2] > UPRIGHT_MIN:
                if self.balanced_stand():
                    self.step(1.0)
                    if self.lying_state() == "up":
                        return True, self.robot.getTime() - t0, attempt
                print("  lost balance after getting up, trying again")
            else:
                self.step(0.5)
        return False, self.robot.getTime() - t0, MAX_GETUP_ATTEMPTS

