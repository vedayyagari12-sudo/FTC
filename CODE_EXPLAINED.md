# Code explained

What every part of this repo does, and how the turret aiming fits into the autos. Written against the code on the `turret-aim` branch (SDK 12.0, Pedro Pathing 3, BIOBUZZ season).

## Contents

1. [Repo map](#1-repo-map)
2. [Field and coordinate rules the code relies on](#2-field-and-coordinate-rules-the-code-relies-on)
3. [Pedro setup (`pedro/`)](#3-pedro-setup-pedro)
4. [Autonomous OpModes](#4-autonomous-opmodes)
5. [TeleOp OpModes and the turret test](#5-teleop-opmodes-and-the-turret-test)
6. [Turret aiming (`TurretAim.java`) and how the autos use it](#6-turret-aiming-turretaimjava-and-how-the-autos-use-it)
7. [Hardware names in one table](#7-hardware-names-in-one-table)
8. [Places where the code disagrees with itself (confirm on the robot)](#8-places-where-the-code-disagrees-with-itself-confirm-on-the-robot)

## 1. Repo map

| Path | What it is |
|---|---|
| `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/` | All of the team's robot code. Everything below is in here unless noted. |
| `.../Close*Alliance1.java`, `.../Far*Alliance1.java` | The four full autos: close and far start, red and blue alliance. They drive, shoot, pick up balls and now aim the turret. |
| `.../Close*AlliancePark.java`, `.../Far*AlliancePark.java` | The four "dead autos": only drive to the parking zone. No shooter, no intake, no turret. |
| `.../RedTeleOp.java` | The main driver-controlled program (red side): Pedro drive, intake, turret auto-aim with AprilTags, hood and flywheel from a shot table. |
| `.../IntakeTeleOp.java` | Older, simpler TeleOp: drives the wheels directly (no Pedro), intake toggle and a ram/push mechanism. |
| `.../Turrettest.java` | Bench-test TeleOp: drive, move the turret with a stick, flywheel, intake and feeding motor. Nothing automatic. |
| `.../TurretAim.java` | New. Pure math that turns the robot pose into a turret angle and servo position. Shared by the four full autos. |
| `.../pedro/Constants.java` | Pedro Pathing 3 configuration for this robot (drive motors, Pinpoint odometry, tuned follower values). |
| `.../pedro/Tuning.java`, `.../pedro/procedures/*` | The official Pedro AutoTune procedures. Only used when tuning, not during a match. |
| `PEDRO_TUNING_CHECKLIST.md` | Step-by-step list for tuning the drivetrain with AutoTune. |
| `FtcRobotController/` | The stock FTC SDK app (samples, main activity). Not edited by the team. |
| `25-26 season/` | Last season's TeleOps and autos, kept for reference. These files are outside `TeamCode`, so they are not built. |
| `website/` | The Torque Tyrants team website (HTML and assets). Not robot code. |
| `_old_project_files_backup/` | Backed-up IDE project files from before the Pedro upgrade. |
| `build*.gradle`, `gradle/`, `libs/` | Build setup: FTC SDK 12.0 and Pedro `revhub:3.0.1`, `tuning:1.0.1`. |
| `.github/` | Contribution and pull request templates that come with the SDK. |

## 2. Field and coordinate rules the code relies on

All the autos and `TurretAim` use the Pedro Pathing 3 frame, in inches and radians:

- Origin is the bottom-left corner of the 144 by 144 inch field.
- The audience wall is at `y = 0`, the red wall at `x = 0`, the blue wall at `x = 144`.
- Heading `0` points toward +x (blue wall) and turning left (counter-clockwise) increases it. Heading `90` faces away from the audience wall.
- The red autos are the blue autos turned 180 degrees about the field center: `x -> 144 - x`, `y -> 144 - y`, heading plus 180. For example Close Blue starts at (83, 136), Close Red at (61, 8).

The HIVE (from the BIOBUZZ Competition Manual TU06, section 9):

- One HIVE structure in the middle of the field, pivot at (72, 72). The red HIVE is centered at `x = 59.25` and the blue HIVE at `x = 84.75` (25.5 inches apart).
- Each HIVE is a seesaw with a CELL at the audience end and one at the far end. Only the raised CELL scores.
- Opening center of a CELL: 16.5 inches from `y = 72` (so `y = 55.5` audience, `y = 88.5` far) and 58.3 inches high. These two numbers are worked out from the manual's drawings, not published, so measure them on a real field.

## 3. Pedro setup (`pedro/`)

`Constants.java` has three blocks that AutoTune generates, plus a factory method:

1. `drivetrainConfig` (Mecanum): motor names `FrontLeft`, `BackLeft`, `FrontRight`, `BackRight`. Left side `REVERSE`, right side `FORWARD`. `manualBrakeMode` is on so the robot holds still when the sticks are released.
2. `localizerConfig` (goBILDA Pinpoint with swing arm pods): device name `pinpoint`, forward pod 10.5 inches to the right of center (`xPodOffset = -10.5`), strafe pod centered front to back (`yPodOffset = 0`), distances in inches. These offsets define the point the robot pose is measured at, so they are already inside every `follower.pose()`.
3. `foresightConfig`: the tuned braking, translational and heading values and the maximum speeds.
4. `Constants.create(hardwareMap)`: builds the `Follower` from the three blocks. Every OpMode calls this once in `init()`.

`Tuning.java` registers four AutoTune procedures (Mecanum, Pinpoint, Foresight, Tests) that you run from a laptop at `http://192.168.43.1:10158`. The other files in `pedro/procedures/` are the official tuners they use.

## 4. Autonomous OpModes

### 4.1 Full autos (`Close/Far` + `Red/Blue` + `Alliance1`)

All four are state machines that run inside `loop()`. Each loop they call `follower.update()` (Pedro moves the robot), then act on the current state. Positions and times are constants at the top of each file.

**Close autos** (start next to the HIVE side of the field):

1. Start pose is set, flywheel starts to rev at 2200 RPM. State `REV_WAIT`: wait until the flywheel is within 100 RPM of 2200, but at least 0.8 s and at most 3 s.
2. Path 1: a short straight line to the shooting spot. Then the feeding motor runs for 2 s to shoot, and the flywheel stops.
3. Path 2: drive to the flower. There the intake turns on, the robot moves 2 inches left then 2 inches forward (from the robot's point of view) and keeps intaking until 3 s after the intake started.
4. Path 3: drive to the end spot, wait 200 ms, back up 3 inches, done. Pedro keeps holding the last pose.

**Far autos** are the same idea with four paths: wait 5 s first so the alliance partner shoots, rev the flywheel, path 1 and shoot (2 s), path 2 to the flower with the robot's back facing it, intake (3 s) while creeping 1 inch forward, path 3 back to the same shooting spot and shoot again (2 s), path 4 to the end spot. The wait after each path is in `WAIT_AFTER_MS = {2000, 3000, 2000, 200}`.

**Helper pieces inside every full auto:**

- `followRobotRelative(forwardIn, leftIn, headingDeg)`: builds a short straight line from where the robot is now, measured forward and left from the robot's own point of view, holding the heading.
- `flywheelRpm()`: converts the flywheel encoder velocity to RPM (28 ticks per revolution is a TODO to confirm for your motor).
- `startFlywheel()` (close autos): sets the flywheel velocity to `-2200 RPM` worth of ticks. The flywheel runs on negative power.
- `stop()`: stops the follower and turns every mechanism off.

**What the turret does in the autos (new, see section 6):** `init()` finds the turret servo and centers it. `start()` and every `loop()` call `aimTurret()`, so the turret keeps pointing at the right HIVE CELL for wherever the robot is while it drives. Telemetry shows the turret angle, servo value, distance to the CELL and an "OUT OF RANGE" warning.

### 4.2 Dead autos (`...AlliancePark`)

Each one waits `START_DELAY_MS`, follows one or two Pedro paths to the parking zone and stops. The close ones curve around the hive support pole first, then drive straight. They use no mechanisms.

## 5. TeleOp OpModes and the turret test

### `RedTeleOp.java` ("Red TeleOp")

- Drive: `follower.manual(y, -x, -rx)`, field position tracked by Pedro. Turn speed is scaled by 0.6.
- Intake: gamepad 2 `B` toggles the intake motor and the two continuous servos.
- Turret and shooter (`updateTurretAndShooter`): works out the angle to a target on the HIVE from the pose, converts it to a servo position (`TURRET_PAN_MIN/MAX_DEG` map to servo 0 to 1) and sets both turret servos. It picks the "back" or "audience" cell by AprilTag ids 0 to 3 or 4 to 7, can be forced with D-pad up/down, and guesses from the field half when no tag is seen. Past the half line a visible tag adds a small correction.
- Shot table: `{distance, RPM, hood}` rows, interpolated by distance, plus a flywheel controller from the Pedro `PIDController`.
- Gamepad 2 left bumper switches between automatic aim and manual (right stick turns the turret, left stick moves the hood). Right trigger is meant to fire but the feed mechanism is still a `TODO`.

### `IntakeTeleOp.java`

An earlier TeleOp: mecanum drive written by hand (no Pedro), `B` toggles the intake, right trigger runs a ram: extends a servo, then drives `shootmotor` forward by 179 encoder ticks per press.

### `Turrettest.java` ("Turret Test")

A bench test with no automation. Gamepad 1 drives. Gamepad 2: right stick X turns the turret (position changes while held and holds when released), `A` toggles the flywheel, D-pad up/down switches between 100 percent and -0.6 power, `B` toggles the intake, right trigger (held fully) runs the feeding motor and intake, `X` recenters the turret. The ramp code is commented out. Use this to check the turret's real travel and direction before trusting any auto-aim (see section 8).

## 6. Turret aiming (`TurretAim.java`) and how the autos use it

`TurretAim.aim(x, y, headingRad, alliance)` takes the numbers straight from `follower.pose()` and returns a `Result`:

1. **Which CELL.** The robot aims at the CELL on the half of the field it stands on: `y < 72` is the audience CELL, otherwise the far CELL. The target is on the alliance's own HIVE: `x = 72 - 12.75` for `RED`, `72 + 12.75` for `BLUE`.
2. **Turret pivot.** The pose is the robot center; the turret pivot is moved by `PIVOT_FORWARD_IN` and `PIVOT_LEFT_IN` (both 0 until measured), rotated by the heading.
3. **Angle.** Field bearing from the pivot to the CELL, minus the robot heading, wrapped to -180 to 180. That is how far the turret must turn from straight ahead (positive is left). It is clamped to `TURRET_LIMIT_DEG` (150) and `outOfRange` says if it had to be.
4. **Servo value.** `TURRET_CENTER + TURRET_SIGN * angle / TURRET_RANGE_DEG`, for `turretServo.setPosition()`.
5. **Distance.** Flat distance from the pivot to the opening center, for the shot table later.

Checked numbers at each full auto's shooting spot (run with `java TurretAim.java`-style self-check and a small driver; red and blue come out mirror-identical):

| Auto | Shooting pose | Target | Turret turn | Servo | Distance |
|---|---|---|---|---|---|
| Close Red | (61, 34), 90 deg | audience CELL (59.25, 55.5) | 4.65 deg left | 0.5155 | 21.6 in |
| Close Blue | (83, 110), 270 deg | far CELL (84.75, 88.5) | 4.65 deg left | 0.5155 | 21.6 in |
| Far Red | (57, 113), 270 deg | far CELL (59.25, 88.5) | 5.25 deg left | 0.5175 | 24.6 in |
| Far Blue | (87, 31), 90 deg | audience CELL (84.75, 55.5) | 5.25 deg left | 0.5175 | 24.6 in |

The robot already faces the HIVE at those spots, so the turret only trims a few degrees. The autos do not check which CELL is currently raised; they always shoot at the CELL on their half.

The same formulas are in the Desmos simulations (`biobuzz-turret-red.html` and `-blue.html`), where you can move the robot and see the angle and distance.

## 7. Hardware names in one table

These must match the Driver Station "Configure Robot" screen exactly.

| Name | Used by | Notes |
|---|---|---|
| `FrontLeft`, `BackLeft`, `FrontRight`, `BackRight` | `pedro/Constants.java`, `IntakeTeleOp` | Drive motors |
| `pinpoint` | `pedro/Constants.java` | goBILDA Pinpoint, I2C port other than 0 |
| `flywheel` | autos, `RedTeleOp`, `Turrettest` | Shooter flywheel motor |
| `intakemotor` | autos, TeleOps | Intake motor |
| `feedingmotor` | autos, `Turrettest` | Feeds balls into the flywheel |
| `turretServo` | autos (new), `Turrettest` | Single turret servo. Name is a placeholder, confirm it |
| `turretServo1`, `turretServo2` | `RedTeleOp` | Two turret servos in that file (see section 8) |
| `hoodServo`, `intakeservo`, `intakeservo2`, `Webcam 1` | `RedTeleOp` | Hood, intake servos, camera |
| `shootmotor`, `intakeservo3` | `IntakeTeleOp` | Ram mechanism |

## 8. Places where the code disagrees with itself (confirm on the robot)

These were already in the code before the turret aiming was added. They decide whether the turret moves the right way and the right amount.

1. **One turret servo or two?** `Turrettest` and the new auto code use one servo named `turretServo`. `RedTeleOp` drives two servos named `turretServo1` and `turretServo2`. Check the robot configuration and fix the name (`TURRET_SERVO_NAME` in each auto) or the TeleOp.
2. **Turret travel.** `Turrettest` assumes 300 degrees of turret per full servo travel (plus or minus 150). `RedTeleOp` maps plus or minus 90 degrees onto the full servo range (180 degrees). `TurretAim.TURRET_RANGE_DEG` follows `Turrettest`. Measure the real travel with `Turrettest` and set the constant.
3. **Direction.** `Turrettest` calls a higher servo position "turret right" (clockwise). `RedTeleOp` makes a left (counter-clockwise) turn a higher servo position. `TurretAim.TURRET_SIGN = 1.0` follows `RedTeleOp`. If the turret swings the wrong way in an auto, set it to `-1.0`.
4. **TeleOp HIVE coordinates.** `RedTeleOp` aims at placeholder points (100, 48) and (100, 73.5) with a half line at 70.75. The manual-based numbers used by `TurretAim` are x = 59.25 (red HIVE), cells at y = 55.5 and 88.5, half line at 72. If you want the TeleOp to share `TurretAim`, that is a small follow-up change.
5. **Turret pivot and shooter height.** `PIVOT_FORWARD_IN`, `PIVOT_LEFT_IN` and `SHOOTER_Z_IN` in `TurretAim` are placeholders (0, 0, 12). Measure them on the robot.
6. **Park auto comments.** The header comments of `CloseRedAlliancePark` (start at 61, 136 facing -90) and `FarRedAlliancePark` (start at 64, 2 facing 90) do not match their constants (61, 8 facing 90, and 64, 142 facing 270). The constants are what runs.
7. **Not compiled here.** The turret changes were checked with a Java self-check of the math only. Build the project in Android Studio before taking it to a match.
