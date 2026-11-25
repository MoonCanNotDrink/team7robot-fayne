"""Reactive NAO goalkeeper for the kick_recovery Webots world.

The goalie reads the simulated ball's Supervisor position and velocity, predicts
where it will cross the goal line, sidesteps toward that intercept, then kicks
the ball away when it is close or blocks it otherwise. This is ground-truth
simulation sensing; it does not use the camera.

Falls: every motion is watched by the same fall check as the field players.
When the goalie falls it braces, gets up (recover_goalie: StandUpFromFront,
GetUpBack or GetUpSide with a slowed, balanced rise, then the shared
RecoveryMixin.balanced_stand), turns back to face the field and returns to the
ready stance.

controllerArgs: "right" (default) defends the goal at x = +4.5 and faces -x;
"left" defends the goal at x = -4.5 and faces +x.
"""
import math
import os
import sys

from controller import Supervisor

# Share the field players' motion, balance and recovery code.
sys.path.insert(0, os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "kick_recovery")))
import poses  # noqa: E402
from balance import Balancer  # noqa: E402
from motion_player import MotionPlayer, load_motion_file, motion_to_sequence  # noqa: E402
from recovery import RecoveryMixin  # noqa: E402
from settings import UPRIGHT_MIN  # noqa: E402


GOAL_LINE_X = 4.0           # distance of the goalie's line from the centre (m)
GOAL_HALF_WIDTH = 1.30
TRIGGER_TIME = 2.5
MIN_TOWARD_SPEED = 0.12
STEP_MS = 20
LATERAL_TOLERANCE = 0.20
KICK_RANGE = 0.45
# Side steps were played at time scale 0.65 (about 1.5x the recorded speed) and the
# goalie fell over. 1.0 is the recorded speed; lower is faster but less stable.
SIDE_STEP_TIME_SCALE = 1.0  # TUNE
SIDE_STEP_WIDTH = 0.9       # scales hip AND ankle roll together so the sole lands flat
MAX_STEPS_PER_SAVE = 8      # never side-step forever (e.g. if the steps make no progress)
BLOCK_TRANSITION_TIME = 0.25  # TUNE was 0.10: snapping into the lean knocked the goalie over
BLOCK_HOLD_TIME = 0.65
READY_TRANSITION_TIME = 0.30
HEADING_TOL = math.radians(20)  # re-face the field after a get-up if further off than this
# Get-up (tested in Webots, see README). Which clip for which lying position:
#   front: Webots' StandUpFromFront (GetUpFront ends in a head-down pike and falls flat)
#   back:  GetUpBack (poses.rollover_sequence never rolled the goalie onto its front)
#   side:  GetUpSide, then the clip for whichever side it ends on
# Both StandUpFromFront and GetUpBack bring the goalie upright and then let it keep
# rotating backwards until it lands on its back. So from the "rise" (feet under the
# body) to the end of the clip the goalie plays the clip GETUP_TAIL_STRETCH times
# slower, leans GETUP_LEAN forwards and runs the Balancer. In 24 trials this got the
# goalie up within 45 s 13 times (front 4/8, back 7/8, side 2/8) against ~1 in 10 before.
GETUP_RISE = {"StandUpFromFront": 2.6, "GetUpBack": 4.0}  # clip time (s) where the rise starts
GETUP_TAIL_STRETCH = 3.0       # TUNE play the rise this many times slower
GETUP_LEAN = 0.10              # TUNE rad added forwards to Ankle/HipPitch during the rise
RISE_BALANCE_GAIN = 1.0        # TUNE Balancer gain during the rise (0 = off)
MAX_GOALIE_GETUP_ATTEMPTS = 4
SIDE_STEP_MOTIONS = ("SideStepLeft", "SideStepRight")
TURN_MOTIONS = ("TurnLeft40", "TurnRight40")
GET_UP_MOTIONS = ("GetUpBack", "GetUpFront", "GetUpSide", "StandUpFromFront")
GOALIE_MOTIONS = (*SIDE_STEP_MOTIONS, "Shoot", *TURN_MOTIONS, *GET_UP_MOTIONS)

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

BLOCK_LEAN = 0.15           # TUNE hip roll of the block lean (was 0.18 hip / 0.12 ankle)


def block_pose(world_y, goal=1.0):
    """Make a wide, low block and lean toward the predicted world-space y.

    goal is +1 for the right goal (goalie faces -x) and -1 for the left goal
    (goalie faces +x). Hip and ankle roll are equal and opposite, so the lean
    moves the body sideways while both soles stay flat on the ground.
    """
    # Facing -x the goalie's local left/right is reversed in world y; facing +x it is not.
    local_y = -goal * world_y
    side = 1.0 if local_y > 0 else -1.0
    pose = dict(READY)
    pose.update({
        "HeadYaw": side * 0.35,
        "LShoulderPitch": 0.55, "LShoulderRoll": 0.85,
        "LElbowYaw": -1.25, "LElbowRoll": -0.95,
        "RShoulderPitch": 0.55, "RShoulderRoll": -0.85,
        "RElbowYaw": 1.25, "RElbowRoll": 0.95,
        "LHipRoll": side * BLOCK_LEAN, "RHipRoll": side * BLOCK_LEAN,
        "LAnkleRoll": -side * BLOCK_LEAN, "RAnkleRoll": -side * BLOCK_LEAN,
        "LKneePitch": 1.0, "RKneePitch": 1.0,
    })
    return pose


class Goalie(RecoveryMixin):
    def __init__(self, goal_side="right"):
        self.goal = -1.0 if goal_side == "left" else 1.0     # +1: right goal, -1: left goal
        self.line_x = self.goal * GOAL_LINE_X
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
        self.robot.step(self.ts)            # sensors hold real values after one step
        self.player = MotionPlayer(self.robot, self.motors, self.sensors, self.ts)
        self.player.target = self.player.current_pose()
        self._tilt_steps = 0                # state for RecoveryMixin.fall_check
        self._prev_uz = (1.0, self.robot.getTime())
        print(f"Goalie active: tracking simulated ball and defending {goal_side} goal.")

    def _load_motions(self):
        here = os.path.dirname(os.path.abspath(__file__))
        webots_home = os.environ.get("WEBOTS_HOME", "")
        directories = [
            os.path.abspath(os.path.join(here, "..", "..", "motions")),
            os.path.join(webots_home, "projects", "robots", "softbank", "nao", "motions"),
            r"C:\Program Files\Webots\projects\robots\softbank\nao\motions",
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
                  "- those moves (or get-ups) will be skipped.")
        return motions

    # ------------------------------------------------------------ helpers used by RecoveryMixin
    def step(self, seconds=None):
        n = 1 if seconds is None else max(1, int(seconds * 1000 / self.ts))
        for _ in range(n):
            if self.robot.step(self.ts) == -1:
                sys.exit(0)

    def stand(self, duration=1.0):
        bal = Balancer(self)
        bal.base_p = bal.base_r = 0.0
        self._tilt_steps = 0
        return self.player.play([(duration, poses.STAND)], check=self.fall_check, feedback=bal)

    def play_motion(self, name, time_scale=1.0, stop_after=None, abort_on_fall=True):
        """Play a .motion file; returns 'done', 'aborted', 'quit' or 'missing'.
        Get-up motions are played with abort_on_fall=False."""
        if name not in self.motions:
            print("Goalie motion not available:", name)
            return "missing"
        self._tilt_steps = 0
        seq = motion_to_sequence(self.motions[name], time_scale, blend_in=0.3,
                                 stop_after=stop_after)
        res = self.player.play(seq, check=self.fall_check if abort_on_fall else None,
                               ease=False)
        if res == "aborted" and abort_on_fall:
            self.brace_for_fall()
        return res

    # ------------------------------------------------------------ goalie moves
    def move_to(self, pose, duration):
        """Blend to a pose with balance control; 'done', 'aborted' (fell) or 'quit'."""
        bal = Balancer(self)
        bal.base_p = bal.base_r = 0.0
        self._tilt_steps = 0
        res = self.player.play([(duration, pose)], check=self.fall_check, feedback=bal)
        if res == "aborted":
            self.brace_for_fall()
        return res

    def side_step(self, name):
        """One complete side-step clip at a stable speed, roll scaled so the sole lands flat."""
        if name not in self.motions:
            return "missing"
        seq = motion_to_sequence(self.motions[name], time_scale=SIDE_STEP_TIME_SCALE,
                                 blend_in=0.3)
        for _, pose in seq:
            for joint in ("LHipRoll", "RHipRoll", "LAnkleRoll", "RAnkleRoll"):
                if joint in pose:
                    pose[joint] *= SIDE_STEP_WIDTH
        self._tilt_steps = 0
        res = self.player.play(seq, check=self.fall_check, ease=False)
        if res == "aborted":
            self.brace_for_fall()
        return res

    def heading(self):
        R = self.me.getOrientation()
        return math.atan2(R[3], R[0])

    def face_field(self):
        """After a get-up the robot can face anywhere: turn back to face the field."""
        want = math.pi if self.goal > 0 else 0.0
        for _ in range(8):
            err = math.atan2(math.sin(want - self.heading()), math.cos(want - self.heading()))
            if abs(err) <= HEADING_TOL:
                return "done"
            res = self.play_motion("TurnLeft40" if err > 0 else "TurnRight40")
            if res != "done":
                return res
        return "done"

    def play_getup(self, name, mirror=False):
        """Play a get-up clip without fall aborts. For clips in GETUP_RISE the part
        from the rise to the end is slowed, leaned forwards and balance-controlled."""
        frames = self.motions[name]
        seq = motion_to_sequence(frames, blend_in=0.3)
        if mirror:
            seq = poses.mirror_sequence(seq)
        print(f"  playing {name}.motion{' (mirrored)' if mirror else ''}")
        rise = GETUP_RISE.get(name)
        if rise is None:
            return self.player.play(seq, ease=False)
        split = next((i for i, (t, _) in enumerate(frames) if i > 0 and t > rise), len(seq))
        res = self.player.play(seq[:split], ease=False)
        if res == "quit" or split >= len(seq):
            return res
        tail = []
        for d, pose in seq[split:]:
            pose = dict(pose)
            for leg in "LR":
                for joint in ("AnklePitch", "HipPitch"):
                    j = leg + joint
                    if j in pose:
                        pose[j] -= GETUP_LEAN           # more negative = forwards
            tail.append((d * GETUP_TAIL_STRETCH, pose))
        bal = None
        if RISE_BALANCE_GAIN > 0:
            bal = Balancer(self, gain=RISE_BALANCE_GAIN)
            bal.base_p = bal.base_r = 0.0               # balance towards upright
        return self.player.play(tail, ease=False, feedback=bal)

    def recover_goalie(self):
        """Get up from any fall. Returns (success, seconds taken, attempts).

        side  -> GetUpSide (mirrored when lying on the other side), then as below
        back  -> GetUpBack
        front -> StandUpFromFront
        then RecoveryMixin.balanced_stand into the stand pose, as the field players do.
        """
        t0 = self.robot.getTime()
        self.step(0.5)
        if self.lying_state() == "up":
            return True, self.robot.getTime() - t0, 0
        for attempt in range(1, MAX_GOALIE_GETUP_ATTEMPTS + 1):
            state = self.lying_state()
            fz, lz, uz = self.torso_axes()
            print(f"  get-up attempt {attempt}: lying on {state} "
                  f"(torso forward_z={fz:.2f}, left_z={lz:.2f}, up_z={uz:.2f})")
            if state == "up":
                return True, self.robot.getTime() - t0, attempt - 1
            if state == "side" and "GetUpSide" in self.motions:
                # The clip is authored for lying on the right side (left axis up).
                self.play_getup("GetUpSide", mirror=lz < 0)
                self.step(0.3)
                state = self.lying_state()
                print(f"  side clip ended on {state}")
            if state in ("back", "side") and "GetUpBack" in self.motions:
                self.play_getup("GetUpBack")
            elif state == "front":
                name = "StandUpFromFront" if "StandUpFromFront" in self.motions else "GetUpFront"
                if name not in self.motions:
                    print("  no front get-up motion available")
                    break
                self.play_getup(name)
            self.step(0.3)
            if self.torso_axes()[2] > UPRIGHT_MIN:
                if self.balanced_stand():
                    self.step(1.0)
                    if self.lying_state() == "up":
                        return True, self.robot.getTime() - t0, attempt
                print("  lost balance after getting up, trying again")
            else:
                print(f"  still down after the clip ({self.lying_state()})")
        return False, self.robot.getTime() - t0, MAX_GOALIE_GETUP_ATTEMPTS

    def get_up(self):
        """Recover from a fall, face the field again and take the ready stance.
        Returns True when the goalie is back up."""
        print("Goalie is down: getting up.")
        ok, seconds, attempts = self.recover_goalie()
        print(f"Goalie get-up: {'success' if ok else 'failed'} in {seconds:.1f} s "
              f"({attempts} attempts)")
        if not ok:
            return False
        if self.face_field() != "done":
            return not self.is_fallen()
        return self.move_to(READY, READY_TRANSITION_TIME) == "done"

    def ball_intercept(self):
        """Return predicted y and arrival time for a ball approaching the goal."""
        p = self.ball.getPosition()
        v = self.ball.getVelocity()
        g = self.goal
        # toward the goal means moving in +x for the right goal, -x for the left one
        if g * v[0] <= MIN_TOWARD_SPEED or g * p[0] >= g * self.line_x:
            return None
        seconds = (self.line_x - p[0]) / v[0]
        if seconds < 0 or seconds > TRIGGER_TIME:
            return None
        y = p[1] + v[1] * seconds
        if abs(y) > GOAL_HALF_WIDTH:
            return None
        return y, seconds

    def move_laterally_to(self, target_y, arrival):
        """Use complete side-step clips toward the detected ball intercept.
        Returns (target_y, arrival, status) with status 'done', 'fallen' or 'quit'."""
        for _ in range(MAX_STEPS_PER_SAVE):
            delta_y = target_y - self.me.getPosition()[1]
            if abs(delta_y) <= LATERAL_TOLERANCE:
                break
            # Facing -x (right goal) world +y is the goalie's right; facing +x it is its left.
            motion = "SideStepRight" if delta_y * self.goal > 0 else "SideStepLeft"
            print(f"Goalie sidestepping {'right' if motion == 'SideStepRight' else 'left'} "
                  f"toward y={target_y:.2f} m.")
            res = self.side_step(motion)
            if res == "quit":
                return target_y, arrival, "quit"
            if res == "aborted" or self.is_fallen():
                return target_y, arrival, "fallen"
            if res == "missing":
                break
            intercept = self.ball_intercept()
            if intercept is None:
                break
            target_y, arrival = intercept
        return target_y, arrival, "done"

    def can_kick_back(self, arrival):
        """Kick a low incoming ball only when it is within the striker's reach."""
        if "Shoot" not in self.motions or arrival <= 0.0:
            return False
        ball = self.ball.getPosition()
        goalie = self.me.getPosition()
        distance = math.hypot(ball[0] - goalie[0], ball[1] - goalie[1])
        return (self.goal * (ball[0] - goalie[0]) <= 0.08     # ball in front of the goalie
                and ball[2] <= 0.35
                and distance <= KICK_RANGE)

    def run(self):
        ready = False
        blocking_until = 0.0
        if self.move_to(READY, READY_TRANSITION_TIME) == "quit":
            return
        while self.robot.step(self.ts) != -1:
            # Down (e.g. hit by the ball or a player): get up before anything else.
            if self.fall_check() or self.is_fallen():
                self.get_up()
                ready = False
                continue
            now = self.robot.getTime()
            intercept = self.ball_intercept()
            if intercept is not None and now >= blocking_until:
                y, arrival = intercept
                y, arrival, status = self.move_laterally_to(y, arrival)
                if status == "quit":
                    break
                if status == "fallen":
                    continue                        # the next loop gets up
                if self.can_kick_back(arrival):
                    print(f"Ball close: kicking it back (arrives in {arrival:.2f}s).")
                    res = self.play_motion("Shoot")
                else:
                    side = ("right" if self.goal * (y - self.me.getPosition()[1]) >= 0
                            else "left")
                    print(f"Ball detected, blocking to {side} "
                          f"(arrives in {arrival:.2f}s).")
                    res = self.move_to(block_pose(y, self.goal), BLOCK_TRANSITION_TIME)
                if res == "quit":
                    break
                blocking_until = self.robot.getTime() + BLOCK_HOLD_TIME
                ready = False
            elif now >= blocking_until and not ready:
                res = self.move_to(READY, READY_TRANSITION_TIME)
                if res == "quit":
                    break
                ready = res == "done"


if __name__ == "__main__":
    Goalie(sys.argv[1] if len(sys.argv) > 1 else "right").run()
