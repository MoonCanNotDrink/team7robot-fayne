"""Reactive NAO goalkeeper for the kick_recovery Webots world.

The goalie reads the simulated ball's Supervisor position and velocity, predicts
where it will cross the goal line, sidesteps toward that intercept, then kicks
the ball away when it is close or blocks it otherwise. This is ground-truth
simulation sensing; it does not use the camera.
"""
import math
import os
import sys

from controller import Supervisor

sys.path.insert(0, os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..")))
from kick_recovery.motion_player import (
    MotionPlayer, load_motion_file, motion_to_sequence)


GOAL_LINE_X = 4.0
GOAL_HALF_WIDTH = 1.30
TRIGGER_TIME = 2.5
MIN_TOWARD_SPEED = 0.12
STEP_MS = 20
LATERAL_TOLERANCE = 0.20
KICK_RANGE = 0.45
MIN_KICK_TIME = 0.25
BLOCK_TRANSITION_TIME = 0.10
SIDE_STEP_MOTIONS = ("SideStepLeft", "SideStepRight")
GOALIE_MOTIONS = (*SIDE_STEP_MOTIONS, "Shoot")

READY = {
    "HeadYaw": 0.0, "HeadPitch": -0.12,
    "LShoulderPitch": 1.15, "LShoulderRoll": 0.48,
    "LElbowYaw": -1.20, "LElbowRoll": -0.50, "LWristYaw": 0.0,
    "RShoulderPitch": 1.15, "RShoulderRoll": -0.48,
    "RElbowYaw": 1.20, "RElbowRoll": 0.50, "RWristYaw": 0.0,
    "LHipYawPitch": 0.0, "LHipRoll": 0.0, "LHipPitch": -0.42,
    "LKneePitch": 0.84, "LAnklePitch": -0.42, "LAnkleRoll": 0.0,
    "RHipYawPitch": 0.0, "RHipRoll": 0.0, "RHipPitch": -0.42,
    "RKneePitch": 0.84, "RAnklePitch": -0.42, "RAnkleRoll": 0.0,
}


def block_pose(world_y):
    """Make a wide, low block and lean toward the predicted world-space y."""
    # The goalie faces -x, so its local left/right is reversed in world y.
    side = -1.0 if world_y >= 0 else 1.0
    pose = dict(READY)
    pose.update({
        "HeadYaw": side * 0.35,
        "LShoulderPitch": 0.55, "LShoulderRoll": 0.85,
        "LElbowYaw": -1.25, "LElbowRoll": -0.95,
        "RShoulderPitch": 0.55, "RShoulderRoll": -0.85,
        "RElbowYaw": 1.25, "RElbowRoll": 0.95,
        "LHipRoll": side * 0.18, "RHipRoll": side * 0.18,
        "LAnkleRoll": -side * 0.12, "RAnkleRoll": -side * 0.12,
        "LKneePitch": 1.0, "RKneePitch": 1.0,
    })
    return pose


class Goalie:
    def __init__(self):
        self.robot = Supervisor()
        self.me = self.robot.getSelf()
        self.ts = int(self.robot.getBasicTimeStep())
        if self.ts <= 0:
            self.ts = STEP_MS
        self.ball = self.robot.getFromDef("BALL")
        if self.ball is None:
            raise RuntimeError("Goalie needs a Solid with DEF BALL in the world.")
        self.motions = self._load_motions()
        self.motors = {}
        self.sensors = {}
        for name in READY:
            motor = self.robot.getDevice(name)
            if motor is None:
                continue
            motor.setVelocity(motor.getMaxVelocity())
            self.motors[name] = motor
            sensor = self.robot.getDevice(name + "S")
            if sensor is not None:
                sensor.enable(self.ts)
                self.sensors[name] = sensor
        if self.robot.step(self.ts) == -1:
            return
        self.current = {}
        for name, motor in self.motors.items():
            sensor = self.sensors.get(name)
            value = sensor.getValue() if sensor else 0.0
            if not math.isfinite(value):
                value = 0.0
            self.current[name] = value
            motor.setPosition(value)
        self.player = MotionPlayer(self.robot, self.motors, self.sensors, self.ts)
        self.player.target = dict(self.current)
        print("Goalie active: tracking simulated ball and defending right goal.")

    def _load_motions(self):
        here = os.path.dirname(os.path.abspath(__file__))
        webots_home = os.environ.get("WEBOTS_HOME", "")
        directories = [
            os.path.abspath(os.path.join(here, "..", "..", "motions")),
            os.path.join(webots_home, "projects", "robots", "softbank", "nao", "motions"),
            "/usr/local/webots/projects/robots/softbank/nao/motions",
        ]
        motions = {}
        for name in GOALIE_MOTIONS:
            for directory in directories:
                path = os.path.join(directory, name + ".motion")
                if os.path.isfile(path):
                    motions[name] = load_motion_file(path)[1]
                    break
        missing = [name for name in GOALIE_MOTIONS if name not in motions]
        if missing:
            print("Goalie motions unavailable:", ", ".join(missing),
                  "- side-stepping or kicking will be skipped when unavailable.")
        return motions

    def ball_intercept(self):
        """Return predicted y and arrival time for a ball approaching the goal."""
        p = self.ball.getPosition()
        v = self.ball.getVelocity()
        if v[0] <= MIN_TOWARD_SPEED or p[0] >= GOAL_LINE_X:
            return None
        seconds = (GOAL_LINE_X - p[0]) / v[0]
        if seconds < 0 or seconds > TRIGGER_TIME:
            return None
        y = p[1] + v[1] * seconds
        if abs(y) > GOAL_HALF_WIDTH:
            return None
        return y, seconds

    def move_to(self, pose, duration):
        start = dict(self.current)
        steps = max(1, int(duration * 1000 / self.ts))
        for k in range(1, steps + 1):
            alpha = 0.5 - 0.5 * math.cos(math.pi * k / steps)
            for name, motor in self.motors.items():
                target = pose.get(name, start[name])
                value = start[name] + alpha * (target - start[name])
                motor.setPosition(value)
            if self.robot.step(self.ts) == -1:
                return False
        for name in self.current:
            self.current[name] = pose.get(name, start[name])
        self.player.target = dict(self.current)
        return True

    def play_motion(self, name):
        """Play a recorded NAO motion and keep the tracked pose in sync."""
        frames = self.motions.get(name)
        if not frames:
            return False
        sequence = motion_to_sequence(frames, blend_in=0.2)
        result = self.player.play(sequence, ease=False)
        self.current = dict(self.player.target)
        return result == "done"

    def move_laterally_to(self, target_y, arrival):
        """Use complete side-step clips toward the detected ball intercept."""
        while abs(target_y - self.me.getPosition()[1]) > LATERAL_TOLERANCE:
            delta_y = target_y - self.me.getPosition()[1]
            # The goalie faces -x, so its local left/right is reversed in world y.
            motion = "SideStepRight" if delta_y > 0 else "SideStepLeft"
            frames = self.motions.get(motion)
            if not frames:
                break
            print(f"Goalie sidestepping {'left' if delta_y < 0 else 'right'} "
                  f"toward y={target_y:.2f} m.")
            if not self.play_motion(motion):
                return target_y, arrival, False
            intercept = self.ball_intercept()
            if intercept is None:
                break
            target_y, arrival = intercept
        return target_y, arrival, True

    def can_kick_back(self, arrival):
        """Kick a low incoming ball only when it is within the striker's reach."""
        if "Shoot" not in self.motions or arrival < MIN_KICK_TIME:
            return False
        ball = self.ball.getPosition()
        goalie = self.me.getPosition()
        distance = math.hypot(ball[0] - goalie[0], ball[1] - goalie[1])
        return (ball[0] <= goalie[0] + 0.08
                and ball[2] <= 0.35
                and distance <= KICK_RANGE)

    def run(self):
        ready = False
        blocking_until = 0.0
        self.move_to(READY, 0.25)
        while self.robot.step(self.ts) != -1:
            now = self.robot.getTime()
            intercept = self.ball_intercept()
            if intercept is not None and now >= blocking_until:
                y, arrival = intercept
                y, arrival, motion_ok = self.move_laterally_to(y, arrival)
                if not motion_ok:
                    break
                if self.can_kick_back(arrival):
                    print(f"Ball close: kicking it back (arrives in {arrival:.2f}s).")
                    if not self.play_motion("Shoot"):
                        break
                else:
                    side = "right" if y >= self.me.getPosition()[1] else "left"
                    print(f"Ball detected, blocking to {side} "
                          f"(arrives in {arrival:.2f}s).")
                    if not self.move_to(block_pose(y), BLOCK_TRANSITION_TIME):
                        break
                blocking_until = self.robot.getTime() + 0.65
                ready = False
            elif now >= blocking_until and not ready:
                if not self.move_to(READY, 0.20):
                    break
                ready = True


if __name__ == "__main__":
    Goalie().run()
