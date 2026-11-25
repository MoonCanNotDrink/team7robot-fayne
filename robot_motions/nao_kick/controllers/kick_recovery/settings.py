"""Shared tuning values for the NAO kick and recovery controller."""
import math

TARGET = (4.5, 0.0)         # world point to kick towards (centre of a goal)
KICK_FOOT = "R"
KICK_STRENGTH = 0.8         # 0..1 for the custom kick
USE_BUILTIN_SHOOT = False   # True = use Webots' Shoot.motion instead of the custom kick

BALL_X = 0.20               # TUNE where the ball should sit in front of the robot (m)
BALL_Y = 0.05               # TUNE sideways offset so the ball is in front of the kicking foot
POS_TOL_X, POS_TOL_Y = 0.05, 0.035
HEAD_TOL = math.radians(20)  # turn clips finish in full steps; allow their small overshoot

# Walking speed: motions are played this many times faster than recorded.
# If the robot falls while walking, the speed-up drops by WALK_SPEED_STEP
# automatically (never below 1.0).
WALK_SPEED = 1.0            # use the recorded gait speed for stability
TURN_SPEED = 0.5            # half-speed turn playback reduces leg clipping and falls
SIDE_STEP_SPEED = 0.6       # complete each lateral step slowly before checking position
SIDE_STEP_WIDTH = 0.9       # reduce lateral reach to keep the stance under the body
WALK_SPEED_STEP = 0.1
WALK_HEAD_TOL = math.radians(25)   # walk without turning first if heading is within this

# Fall detection uses the torso orientation from the Supervisor (no sign guessing).
UPRIGHT_MIN = 0.5           # fallen when the torso's up-axis is tilted more than ~60 deg
FALL_CONFIRM_S = 0.15       # tilt must last this long, so a wobble is not counted as a fall
FALLING_UZ = 0.75           # also "falling" when tilted past ~40 deg AND still tipping over
FALLING_RATE = -1.0         # ... at this rate of torso up_z (1/s); lets the robot brace in time

# Kick and stop stability
UNLOAD_SHARE = 0.15         # kicking foot may carry at most this share of the weight before lifting
UNLOAD_MAX_S = 1.5          # give up growing the weight shift after this long
KICK_ABORT_TILT = 0.25      # TUNE torso tilt (rad) on one foot that aborts the swing and plants the foot
STILL_RATE = 0.3            # body counts as still below this angular speed (rad/s)
STILL_HOLD_S = 0.25         # ... for this long
STILL_MAX_S = 2.0           # longest wait for the body to settle before a kick
WALK_STOP_MAX_S = 0.6       # after reaching a walk target, wait at most this long for both feet down

# Balance after getting up: hold the get-up's last pose, then move slowly to the
# stand pose while an ankle/hip controller corrects forward/backward lean.
SETTLE_S = 1.0              # TUNE hold the get-up motion's final pose this long
TO_STAND_S = 2.0            # TUNE time to move from that pose to the stand pose
BAL_KP_ANKLE = 0.8          # TUNE rad of ankle correction per unit of lean
BAL_KD_ANKLE = 0.08         # TUNE damping on the lean rate
BAL_KP_HIP = 0.5            # TUNE rad of hip correction per unit of lean
BAL_MAX = 0.35              # largest correction allowed (rad)

# Get-up clips (RecoveryMixin.recover, used by field players and the goalie):
#   front: Webots' StandUpFromFront (GetUpFront ends in a head-down pike and falls flat)
#   back:  GetUpBack (poses.rollover_sequence never rolled the robot onto its front)
#   side:  GetUpSide, then the clip for whichever side it ends on
# StandUpFromFront and GetUpBack bring the robot upright and then let it keep rotating
# backwards onto its back. So from the "rise" (feet under the body) to the end of the
# clip it is played slower, leaned forwards and balance-controlled.
GETUP_RISE = {"StandUpFromFront": 2.6, "GetUpBack": 4.0}  # clip time (s) where the rise starts
GETUP_TAIL_STRETCH = 3.0    # TUNE play the rise this many times slower
GETUP_LEAN = 0.10           # TUNE rad added forwards to Ankle/HipPitch during the rise
RISE_BALANCE_GAIN = 1.0     # TUNE balance.Balancer gain during the rise (0 = off)

# Kick types. Pick one with controllerArgs "kick=slow|normal|fast|side" (default KICK_TYPE).
#   strength   0..1: back-swing size and swing speed
#   tempo      scales the back-swing and swing-through only (< 1 faster, > 1 slower); the weight
#              shift and re-centring always run at normal speed because they set the balance
#   ball_x     where the ball sits in front of the robot (m)
#   ball_y     how far the ball sits to the kicking-foot side (m)
#   direction  ball direction relative to the robot's heading for a RIGHT-foot kick
#              (deg, mirrored for the left foot): -90 = to the robot's right
KICK_TYPE = "normal"
KICK_TYPES = {
    "slow":   dict(strength=0.35, tempo=1.5, ball_x=BALL_X, ball_y=BALL_Y, direction=0),  # TUNE
    "normal": dict(strength=KICK_STRENGTH, tempo=1.0, ball_x=BALL_X, ball_y=BALL_Y, direction=0),
    "fast":   dict(strength=1.0, tempo=0.75, ball_x=BALL_X, ball_y=BALL_Y, direction=0),  # TUNE
    "side":   dict(strength=0.8, tempo=1.0, ball_x=0.03, ball_y=0.17, direction=-90),     # TUNE
}

STRENGTHS = [0.2, 0.4, 0.6, 0.8, 1.0]
KICK_COMPARE_TRIALS = 3     # kicks per type in kick_compare mode
KICKS_PER_STRENGTH = 3
GETUP_TRIALS = 10
MAX_GETUP_ATTEMPTS = 4

KICK_PHASES = ["stand", "weight shift", "hold", "lift foot", "back-swing", "swing",
               "hold", "retract", "put foot down", "hold", "re-centre", "hold"]
KICK_STAGE_PHASES = {"shift": KICK_PHASES[0:3], "swing": KICK_PHASES[3:10],
                     "centre": KICK_PHASES[10:12]}

WALK_MOTIONS = ["Forwards", "Forwards50", "Backwards", "SideStepLeft", "SideStepRight",
                "TurnLeft40", "TurnRight40", "TurnLeft60", "TurnRight60",
                "StandUpFromFront", "Shoot",
                # get-up motions from Cyberbotics' NAO wrestling examples (optional, see README)
                "GetUpBack", "GetUpFront", "GetUpSide"]



def wrap(angle):
    """Normalize an angle to the range [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))
