# NAO V5 kicking and recovery controller (Webots)

ENG5325 Stage 1 deliverable: **kicking and recovery controller**.
Python controller for the built-in Webots `Nao` (V5) robot.

## Folder layout
```
nao_kick/
  worlds/kick_recovery_test.wbt        test world: player NAO, goalie NAO, ball, 9 x 6 m pitch
  worlds/match.wbt                     4 v 4 match: per team 3 field players + 1 goalie
  controllers/kick_recovery/
    kick_recovery.py                    main controller (modes below)
    settings.py                         tuning values, kick types   <- tune here
    kicking.py / walking.py / recovery.py   kick + positioning, walking, falls and get-up
    poses.py                            stand pose, kick and roll-over keyframes  <- tune here
    motion_player.py                    plays keyframes and Webots .motion files
    balance.py                          ankle/hip balance controller and foot-sensor reader
  controllers/goalie/goalie.py          ball-tracking goalkeeper with sidesteps, kick and block
  motions/                              copy Webots' NAO motion files here (step 2)
```

## Set-up
1. Unzip and open `worlds/kick_recovery_test.wbt` in Webots (R2025a).
2. Copy all `.motion` files from
   `WEBOTS_HOME/projects/robots/softbank/nao/motions`
   (Windows default: `C:\Program Files\Webots\projects\robots\softbank\nao\motions`)
   into the `motions` folder. The controller uses them for walking, turning,
   side steps and standing up from the front. The console lists which motions it found.
3. Python must be installed on Windows (Webots: Tools > Preferences > Python command).
4. Press Play.

## Modes
Change the NAO's `controllerArgs` field in the scene tree, then reload the world.

| Mode | What it does | Output |
| --- | --- | --- |
| `sensor_check` | Prints the accelerometer while you drag the robot onto its back and front. Run this **first**. | Console |
| `kick_test` | Puts the ball at the kicking foot and kicks at strengths 0.2–1.0, 3 kicks each, with the kick type from `kick=...` (default `normal`). Records peak ball speed, distance rolled, direction error and falls; gets up after falls. | `kick_test.csv` |
| `kick_compare` | Kicks each kick type (slow, normal, fast, side) 3 times at its own strength and logs the same values, to compare them. | `kick_compare.csv` |
| `getup_test` | Lays the robot on its back 10 times at random headings and times the get-up. | `getup_test.csv` |
| `play` | Walks to the ball, lines up with the target (default: right goal centre at x = 4.5 m), kicks, gets up if it falls, repeats. | Console |

Options can be added as extra `controllerArgs` entries, in any order:

| Option | Values | Meaning |
| --- | --- | --- |
| `attack=` | `right` (default), `left` | goal to kick towards: x = +4.5 m or x = -4.5 m |
| `kick=` | `slow`, `normal` (default), `fast`, `side` | kick type used by `play` and `kick_test` |

Example: `controllerArgs ["play", "attack=left", "kick=fast"]`.
The goalie controller takes `"right"` (default) or `"left"` for the goal it defends.

### Kick types
Defined in `KICK_TYPES` in `settings.py`; all use the same closed-loop balance
(weight-shift check, support-ankle balance, abort and plant foot).

| Type | Keyframes | Strength | Swing tempo | Ball position | Ball goes |
| --- | --- | --- | --- | --- | --- |
| `slow` | `poses.kick_phases` | 0.35 | 1.5x slower | in front of the kicking foot | straight ahead, gently (short pass) |
| `normal` | `poses.kick_phases` | 0.8 | normal | in front of the kicking foot | straight ahead |
| `fast` | `poses.kick_phases` | 1.0 | 0.75x (faster) | in front of the kicking foot | straight ahead, hard (shot) |
| `side` | `poses.side_kick_phases` | 0.8 | normal | 3 cm ahead, 17 cm out beside the kicking foot | sideways, away from the support leg (right foot: to the robot's right) |

`tempo` only changes the back-swing and swing-through, never the weight shift or
the time on one foot, so the slow kick is not less stable. For the side kick the
robot lines up side-on to the target (`stance_target`), and the leg swings out
with hip roll while AnkleRoll = -HipRoll keeps the sole flat. There is no inward
back-swing: after the weight shift the feet are only ~10 cm apart.
Tune the side kick with `kick_test` and `kick=side`: adjust `ball_x` / `ball_y`
until the foot meets the ball centre, then `SIDE_KICK_LIFT` and the swing line
marked `TUNE` in `poses.side_kick_phases`.

### Match world
`worlds/match.wbt` has two teams of four, each 3 field players + 1 goalie:

| Team | Robots | Field players | Goalie |
| --- | --- | --- | --- |
| red (default colour) | `NAO`, `RED_2`, `RED_3`, `RED_GOALIE` | `kick_recovery`, `["play"]`, attack the right goal | `goalie`, `["left"]` at x = -4 |
| blue | `BLUE_1`, `BLUE_2`, `BLUE_3`, `GOALIE` | `kick_recovery`, `["play", "attack=left"]` | `goalie`, `["right"]` at x = +4 |

There are no team behaviours yet: every field player runs the same `play` loop,
so all six chase the ball. Use `kick_recovery_test.wbt` for `kick_test`,
`kick_compare` and `getup_test`; those modes move the robot and the ball, which
the other players would interfere with.

The Webots test world has a red player NAO controlled by `kick_recovery` and a
blue goalie NAO in front of the right goal, controlled by `goalie`. The goalie
tracks the simulated ball position and velocity, predicts whether it will cross
the goal mouth up to 2.5 seconds ahead, then immediately uses side-step motions
toward the intercept using faster side-step clips. If the incoming ball is low
and within kicking range, it plays `Shoot.motion` to send it back; otherwise it
spreads its arms and leans toward the intercept to block. The controller commands the NAO's maximum joint
speeds and uses shorter transitions for the block and ready poses. It returns
to a ready stance after the save. This uses Supervisor ground-truth state, not
camera vision.
The world has a 9 m x 6 m marked playing area on 11 m x 8 m turf.
Both goals are 2.8 m wide overall (2.6 m clear between the posts), 1.2 m high,
and use 0.1 m square posts and crossbar, matching `kidsize_pitch.wbt`.

CSV files are written next to `kick_recovery.py`.

## How it works
- **Kick** (`poses.kick_sequence`): stand → shift weight onto the support foot →
  lift → back-swing → swing through → retract → put down → re-centre.
  `strength` (0–1) sets the back-swing size and swing speed.
  Set `USE_BUILTIN_SHOOT = True` to compare with Webots' `Shoot.motion`.
- **Positioning** (`position_for_kick`): go to a point 0.45 m behind the ball
  (round the side if the robot is past it), then turn to face the target and
  correct with side steps and short steps until the ball is at
  (`BALL_X`, `BALL_Y`) in front of the kicking foot.
- **Fall detection**: the torso accelerometer; upright when the vertical axis
  reads more than 5 m/s².
- **Recovery**: on the back or side → roll onto the front (`poses.rollover_sequence`),
  trying the other side if it fails → `StandUpFromFront.motion` → check upright.
  Up to 3 attempts.
- Ball and robot positions come from the Supervisor (ground truth). Replace
  them with camera-based estimates later.

## Stability features
- **Closed-loop kick** (`kick`): the kick runs in three stages (`poses.kick_phases`).
  With the `LFsr` / `RFsr` foot sensors the weight shift starts at `WEIGHT_SHIFT`
  and grows (up to `WEIGHT_SHIFT_MAX`) until the kicking foot carries less than
  `UNLOAD_SHARE` of the weight; only then is the foot lifted.
  Without the sensors the fixed `WEIGHT_SHIFT` is used.
- **Ankle/hip balance** (`balance.Balancer`): during standing, the weight shift and
  the swing, the torso tilt is fed back to AnklePitch/AnkleRoll (and HipPitch)
  of the supporting foot. Gains are at the top of `balance.py`.
- **Kick abort**: if the torso tilts more than `KICK_ABORT_TILT` while on one foot,
  the swing stops and the kicking foot is put straight down (`plant_foot_sequence`).
- **Arm counter-swing**: the arms swing against the kicking leg (`ARM_SWING`)
  and the support-side arm opens slightly for sideways balance.
- **Settle before kicking** (`wait_until_still`): waits until the body has stopped swaying.
- **Clean walk stops**: after reaching a walking target the gait keeps running until
  both feet are on the ground (at most `WALK_STOP_MAX_S`), then blends to the stand pose.
- **Flat side steps**: `SIDE_STEP_WIDTH` now scales AnkleRoll with HipRoll so the sole lands flat.
- **Early fall detection and bracing**: a fall is also detected when the torso is past
  `FALLING_UZ` and still tipping; the robot then takes a protective pose
  (`poses.brace_pose`) before it hits the ground.

## Tuning order
1. `sensor_check`: on the back `ax` should be positive; if not, set `BACK_SIGN = -1`.
2. Kick balance: run `kick_test` at strength 0.2. If the robot falls towards the
   kicking side, increase `WEIGHT_SHIFT`; if it falls towards the support side, decrease it.
   With foot sensors, watch for "kicking foot still carries ..." in the console.
   If the balance correction oscillates, lower `KP_*` / raise `KD_*` in `balance.py`;
   if the robot drifts over slowly, raise `KP_*`. Set `KICK_ABORT_TILT` just above the
   tilt seen in good kicks.
3. Kick power: raise the strengths only once 0.2 is stable; tune the back-swing
   and swing lines marked `TUNE` in `poses.py`.
4. Contact: adjust `BALL_X` / `BALL_Y` until the foot meets the ball centre
   (direction error close to 0°).
5. Get-up: run `getup_test`; tune the roll-over keyframes marked `TUNE`.
   Double-click the robot to open its robot window and read live joint angles.

All keyframe values are first designs and must be tuned in Webots.
