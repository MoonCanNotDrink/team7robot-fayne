"""Kicking and recovery controller for the Webots NAO V5 (ENG5325, Stage 1).

Modes (set in the robot's controllerArgs field in Webots):
  play         walk to the ball, line up, kick towards the target, get up if fallen, repeat
  kick_test    repeat kicks at several strengths and log ball speed, distance, direction, falls
  getup_test   lay the robot on its back again and again and log get-up success and time
  sensor_check print accelerometer readings while you move the robot (to check fall signs)

Needs: the robot's 'supervisor' field set to TRUE (used for ground-truth ball and
robot positions and for resetting tests) and a ball Solid with DEF name BALL.
"""
import csv
import math
import os
import random
import sys

from controller import Supervisor

import poses
from balance import Balancer, FootSensors
from kicking import KickingMixin
from recovery import RecoveryMixin
from settings import *  # shared controller tuning values
from walking import WalkingMixin
from motion_player import MotionPlayer, load_motion_file, motion_to_sequence

def axis_angle_from_yaw_pitch(yaw, pitch):
    """Rotation Rz(yaw) * Ry(pitch) as a Webots axis-angle [x, y, z, angle]."""
    cy, sy, cp, sp = math.cos(yaw), math.sin(yaw), math.cos(pitch), math.sin(pitch)
    R = [[cy * cp, -sy, cy * sp],
         [sy * cp,  cy, sy * sp],
         [-sp,     0.0, cp]]
    ang = math.acos(max(-1.0, min(1.0, (R[0][0] + R[1][1] + R[2][2] - 1) / 2)))
    if abs(ang) < 1e-6:
        return [0, 0, 1, 0]
    if abs(ang - math.pi) < 1e-4:            # 180 deg: axis from the diagonal
        x = math.sqrt(max(0, (R[0][0] + 1) / 2)); y = math.sqrt(max(0, (R[1][1] + 1) / 2))
        z = math.sqrt(max(0, (R[2][2] + 1) / 2))
        return [x, y, z, ang]
    k = 1 / (2 * math.sin(ang))
    return [k * (R[2][1] - R[1][2]), k * (R[0][2] - R[2][0]), k * (R[1][0] - R[0][1]), ang]


class NaoKickRecovery(RecoveryMixin, KickingMixin, WalkingMixin):
    def __init__(self):
        self.robot = Supervisor()
        self.ts = int(self.robot.getBasicTimeStep())
        self.motors, self.sensors = {}, {}
        for name in poses.JOINTS:
            m = self.robot.getDevice(name)
            if m is None:
                continue
            self.motors[name] = m
            ps = self.robot.getDevice(name + "S")
            if ps is not None:
                ps.enable(self.ts)
                self.sensors[name] = ps
        self.acc = self.robot.getDevice("accelerometer")
        self.acc.enable(self.ts)
        self.feet = FootSensors(self.robot, self.ts)
        # one step so the position sensors and accelerometer hold real values (not NaN)
        self.robot.step(self.ts)
        self.player = MotionPlayer(self.robot, self.motors, self.sensors, self.ts)
        self.player.target = self.player.current_pose()

        self.me = self.robot.getSelf()
        self.ball = self.robot.getFromDef("BALL")
        if self.ball is None:
            sys.exit("No Solid with DEF BALL found in the world.")
        self.home = self.me.getPosition()             # standing height comes from the world file
        self.motions = self._load_motions()
        self.log_dir = os.path.dirname(os.path.abspath(__file__))
        self._tilt_steps = 0
        self._prev_uz = (1.0, self.robot.getTime())
        self.walk_speed = WALK_SPEED
        self.turn_speed = TURN_SPEED
        print(f"Motion speeds: walk {self.walk_speed:.1f}x, turn {self.turn_speed:.1f}x")
        print("Foot force sensors:", "found (closed-loop weight shift on)" if self.feet.available
              else "not found (fixed weight shift)")

    # ------------------------------------------------------------ shared helpers
    def _load_motions(self):
        here = os.path.dirname(os.path.abspath(__file__))
        dirs = [os.path.join(here, "..", "..", "motions"),
                os.path.join(os.environ.get("WEBOTS_HOME", ""), "projects", "robots", "softbank", "nao", "motions"),
                r"C:\Program Files\Webots\projects\robots\softbank\nao\motions",
                "/usr/local/webots/projects/robots/softbank/nao/motions"]
        found = {}
        for name in WALK_MOTIONS:
            for d in dirs:
                p = os.path.join(d, name + ".motion")
                if os.path.isfile(p):
                    found[name] = load_motion_file(p)[1]
                    break
        missing = [n for n in WALK_MOTIONS if n not in found]
        print("Loaded motions:", ", ".join(found) or "none")
        if missing:
            print("Missing motions:", ", ".join(missing),
                  "- copy WEBOTS_HOME/projects/robots/softbank/nao/motions into this project.")
        return found

    def step(self, seconds=None):
        n = 1 if seconds is None else max(1, int(seconds * 1000 / self.ts))
        for _ in range(n):
            if self.robot.step(self.ts) == -1:
                sys.exit(0)

    def robot_pose(self):
        p = self.me.getPosition()
        R = self.me.getOrientation()
        return p[0], p[1], math.atan2(R[3], R[0])

    def ball_pos(self):
        b = self.ball.getPosition()
        return b[0], b[1]

    def ball_speed(self):
        v = self.ball.getVelocity()
        return math.hypot(v[0], v[1])

    def play_motion(self, name, time_scale=1.0, stop_after=None, abort_on_fall=True):
        """Play a .motion file. Get-up motions must use abort_on_fall=False,
        otherwise they stop immediately because the robot is lying down."""
        if name not in self.motions:
            print("Motion not available:", name)
            self.step(0.5)
            return "missing"
        check = self.fall_check if abort_on_fall else None
        self._tilt_steps = 0
        seq = motion_to_sequence(self.motions[name], time_scale, stop_after=stop_after)
        res = self.player.play(seq, check=check)
        if res == "done" and stop_after is not None:
            res = self.player.play([(0.5, poses.STAND)], check=check)
        if res == "aborted" and abort_on_fall:
            self.brace_for_fall()
        return res

    def stand(self, duration=1.0):
        """Blend to the stand pose with ankle/hip balance aiming for an upright torso."""
        bal = Balancer(self)
        bal.base_p = bal.base_r = 0.0
        self._tilt_steps = 0
        return self.player.play([(duration, poses.STAND)], check=self.fall_check, feedback=bal)

    # ------------------------------------------------------------ recovery
    # ------------------------------------------------------------ test set-up
    def reset_robot(self, x=0.0, y=0.0, yaw=0.0, on_back=False):
        self.player.play([(0.05, poses.LYING if on_back else poses.STAND)])
        z = 0.12 if on_back else self.home[2]
        self.me.getField("translation").setSFVec3f([x, y, z])
        self.me.getField("rotation").setSFRotation(
            axis_angle_from_yaw_pitch(yaw, -math.pi / 2 if on_back else 0.0))
        self.me.resetPhysics()
        self.step(1.5)

    def place_ball(self, x, y):
        self.ball.getField("translation").setSFVec3f([x, y, 0.07])
        self.ball.resetPhysics()
        self.step(0.3)

    def csv_writer(self, name, header):
        f = open(os.path.join(self.log_dir, name), "w", newline="")
        w = csv.writer(f)
        w.writerow(header)
        print("Logging to", f.name)
        return f, w

    # ------------------------------------------------------------ modes
    def run_play(self):
        self.stand(1.0)
        kicks = 0
        while True:
            if self.is_fallen():
                ok, t, n = self.recover()
                print(f"Recovered: {ok} in {t:.1f} s ({n} attempts)")
                continue
            ready = self.ready_to_kick()
            if ready:
                print("Ball already in front and aligned; kicking without repositioning.")
            if ready or self.position_for_kick():
                if not self.wait_until_still():
                    continue
                speed, fell = self.kick()
                kicks += 1
                print(f"Kick {kicks}: peak ball speed {speed:.2f} m/s, fell: {fell}")
                self.step(1.0)
                bx, by = self.ball_pos()
                if abs(bx) > 4.5 and abs(by) < 1.3:
                    print("GOAL - resetting the ball")
                    self.place_ball(random.uniform(-1, 1), random.uniform(-1.5, 1.5))

    def run_kick_test(self):
        self.stand(1.0)
        f, w = self.csv_writer("kick_test.csv",
                               ["kick", "strength", "trial", "peak_speed_mps", "distance_m",
                                "direction_error_deg", "fell", "recovered", "recovery_s"])
        side = -1.0 if KICK_FOOT == "R" else 1.0
        labels = ["builtin"] if USE_BUILTIN_SHOOT else STRENGTHS
        for s in labels:
            for trial in range(1, KICKS_PER_STRENGTH + 1):
                self.reset_robot(0, 0, 0)
                self.place_ball(BALL_X, side * BALL_Y)
                self.wait_until_still()
                b0 = self.ball_pos()
                speed, fell = self.kick(strength=0 if s == "builtin" else s)
                self.step(3.0)                               # let the ball roll
                b1 = self.ball_pos()
                dist = math.hypot(b1[0] - b0[0], b1[1] - b0[1])
                derr = math.degrees(math.atan2(b1[1] - b0[1], b1[0] - b0[0])) if dist > 0.02 else float("nan")
                rec, rt = "", ""
                if fell or self.is_fallen():
                    ok, t, _ = self.recover()
                    rec, rt = ok, round(t, 2)
                w.writerow(["custom" if s != "builtin" else "Shoot.motion", s, trial,
                            round(speed, 3), round(dist, 3), round(derr, 1),
                            fell, rec, rt])
                f.flush()
                print(f"strength {s} trial {trial}: {speed:.2f} m/s, {dist:.2f} m, "
                      f"direction {derr:.1f} deg, fell {fell}")
        f.close()
        print("Kick test finished.")

    def run_getup_test(self):
        f, w = self.csv_writer("getup_test.csv", ["trial", "success", "time_s", "attempts"])
        wins = 0
        for trial in range(1, GETUP_TRIALS + 1):
            self.reset_robot(0, 0, random.uniform(-math.pi, math.pi), on_back=True)
            print("Lying state after reset:", self.lying_state())
            ok, t, n = self.recover()
            wins += ok
            w.writerow([trial, ok, round(t, 2), n]); f.flush()
            print(f"Get-up trial {trial}: success {ok}, {t:.1f} s, {n} attempts")
        f.close()
        print(f"Get-up test finished: {wins}/{GETUP_TRIALS} successful.")

    def run_sensor_check(self):
        self.stand(1.0)
        print("Every 0.5 s: accelerometer and torso orientation. Drag the robot onto its")
        print("back and front (Shift + drag) and check the state reads back / front / up.")
        while True:
            self.step(0.5)
            a = self.acc_values()
            fz, lz, uz = self.torso_axes()
            print(f"acc = [{a[0]:6.2f} {a[1]:6.2f} {a[2]:6.2f}]   torso up_z = {uz:5.2f}  "
                  f"forward_z = {fz:5.2f}  ->  {self.lying_state()}")


if __name__ == "__main__":
    mode = sys.argv[1] if len(sys.argv) > 1 else "play"
    nao = NaoKickRecovery()
    {"play": nao.run_play, "kick_test": nao.run_kick_test,
     "getup_test": nao.run_getup_test, "sensor_check": nao.run_sensor_check}.get(mode, nao.run_play)()
