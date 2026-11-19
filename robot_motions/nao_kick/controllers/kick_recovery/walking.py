"""Recorded walking and turning behavior for the NAO controller."""

import math
import sys

import poses
from motion_player import motion_to_sequence
from settings import *  # shared walking and heading tuning values


class WalkingMixin:
    def double_support(self):
        """True when both feet carry weight (foot sensors), or, without them,
        when both knees are bent about the same (neither leg is swinging)."""
        share = self.feet.share("L")
        if share is not None:
            return 0.3 < share < 0.7
        pose = self.player.current_pose()
        return abs(pose.get("LKneePitch", 0.0) - pose.get("RKneePitch", 0.0)) < 0.1

    def play_until(self, name, done, max_time=12.0, speed=None):
        """Loop a walking motion until done() is True, then return to the stand pose.

        Stops on the robot's actual position, not on time, because the first
        second of a walking motion is mostly the gait starting up.
        Returns 'done', 'fallen', 'timeout' or 'missing'.
        """
        if name not in self.motions:
            print("Motion not available:", name)
            self.step(0.5)
            return "missing"
        self._tilt_steps = 0
        t_end = self.robot.getTime() + max_time
        reached = [False]
        reached_at = [None]

        def check():
            if self.fall_check():
                return True
            now = self.robot.getTime()
            if reached_at[0] is None and done():
                reached[0] = True
                reached_at[0] = now
            if reached_at[0] is not None:
                # do not stop with a foot in the air: wait for both feet down
                return self.double_support() or now - reached_at[0] > WALK_STOP_MAX_S
            return now > t_end

        is_turn = name.startswith("Turn")
        if speed is None:
            speed = self.turn_speed if is_turn else self.walk_speed
        seq = motion_to_sequence(self.motions[name], time_scale=1.0 / speed, blend_in=0.2)
        while True:
            res = self.player.play(seq, check=check, ease=False)
            if res == "quit":
                sys.exit(0)
            # `check` aborts the current motion both when the target is reached
            # and when the robot falls. Replaying after `done()` caused endless
            # start-stop walking at the target until the timeout.
            if reached[0] or res == "aborted" or self.robot.getTime() > t_end:
                break
        if self.is_fallen():
            # walking too fast for the robot: slow down for the rest of the run
            if is_turn and self.turn_speed > 1.0:
                self.turn_speed = max(1.0, self.turn_speed - WALK_SPEED_STEP)
                print(f"  fell while turning: turn speed-up lowered to {self.turn_speed:.1f}")
            elif not is_turn and self.walk_speed > 1.0:
                self.walk_speed = max(1.0, self.walk_speed - WALK_SPEED_STEP)
                print(f"  fell while walking: walk speed-up lowered to {self.walk_speed:.1f}")
            self.brace_for_fall()
            return "fallen"
        self.stand(0.5)
        result = "done" if reached[0] else "timeout"
        x, y, yaw = self.robot_pose()
        print(f"  {name:13s} -> {result:7s} robot at ({x:5.2f}, {y:5.2f}) facing {math.degrees(yaw):6.1f} deg")
        return result

    def turn_to(self, heading, tol=HEAD_TOL):
        """Turn using complete recorded steps so the legs do not stop mid-stride."""
        for _ in range(12):
            current_heading = self.robot_pose()[2]
            err = wrap(heading - current_heading)
            if abs(err) <= tol:
                return "done"
            large_turn = abs(err) > math.radians(50)
            if large_turn and "TurnLeft60" in self.motions and "TurnRight60" in self.motions:
                name = "TurnLeft60" if err > 0 else "TurnRight60"
            else:
                name = "TurnLeft40" if err > 0 else "TurnRight40"
            print(f"  {name}: heading {math.degrees(current_heading):.1f} deg "
                  f"toward {math.degrees(heading):.1f} deg")
            # Do not interrupt the clip when the heading crosses the target. The
            # clip ends with a stable stance; the next iteration checks the angle.
            res = self.play_motion(name, time_scale=1.0 / self.turn_speed,
                                   abort_on_fall=True)
            if self.is_fallen():
                return "fallen"
            if res != "done":
                return res
        return "timeout"

    def walk_to(self, gx, gy, tol=0.08):
        """Walk forwards to a point (turning towards it first)."""
        x, y, yaw = self.robot_pose()
        if math.hypot(gx - x, gy - y) <= tol:
            return "done"
        # Finish the turn before starting a walking motion. The wider walking
        # tolerance made the robot walk diagonally while it was still reorienting.
        res = self.turn_to(math.atan2(gy - y, gx - x), tol=HEAD_TOL)
        if res != "done":
            return res

        def done():
            ex, ey, d = self.errors_to(gx, gy)
            # reached, passed it, or drifted off line (then turn again next time)
            return d < tol or ex < 0.0 or abs(math.atan2(ey, max(ex, 1e-3))) > math.radians(35)
        far = math.hypot(gx - x, gy - y) > 0.8 and "Forwards50" in self.motions
        return self.play_until("Forwards50" if far else "Forwards", done)

