# Pedro Pathing 3 - Drivetrain Setup Checklist (BIOBUZZ robot)

Everything in code is already done. This file is **only the stuff you have to do**.
Scope: drivetrain + path following only (goBILDA Pinpoint with **Swing Arm** pods). Turret/shooter not included.

---------------------------------------------------------------------------
## What was changed in this project (for reference)
- Upgraded build files from FTC SDK 11.2 -> **12.0** (Pedro 3.0.1 is built against 12.0).
- Added Pedro dependencies (`revhub:3.0.1`, `tuning:1.0.1`) + maven repo in `build.dependencies.gradle`.
- Added `TeamCode/.../teamcode/pedro/` : `Constants.java`, `Tuning.java`, `procedures/*` (official tuners).
- `Constants.java` is pre-filled: drive motors = `FrontLeft`, `FrontRight`, `BackLeft`, `BackRight`
  (read from your teleop), Pinpoint = Swing Arm pods, name `pinpoint` (**placeholder**).
- `RedTeleOp.java` ported to the Pedro 3 API (details at the bottom). `IntakeTeleOp.java` untouched.
- Originals of every file I overwrote are in `_pedro_setup_backup/` (same folder layout).

---------------------------------------------------------------------------
## PART 1 - On your computer (Android Studio)  ~10 min
1. Open this folder in Android Studio and click **Sync Now**.
   - First sync downloads Gradle 9.1 + Pedro libraries: needs internet.
   - If you get an AGP / "Android Studio too old" error: update Android Studio
     (needs **Narwhal 3 Feature Drop or newer** for AGP 8.13.2). Do NOT downgrade the AGP.
2. **Build > Make Project**. It should be error free. If anything is red, copy the error to me.

## PART 2 - Robot hardware (YOU must do these)
3. **Update the Control Hub and Driver Hub apps to FTC 12.0.** The robot app version must match
   the SDK in the code, or you get a version-mismatch error / things behave oddly.
4. **Pinpoint wiring**
   - Plug into an I2C port **other than port 0** (port 0 is the hub's internal IMU).
   - Sticker side (with the ports) facing **up**.
   - **Forward** pod -> **X** port, **strafe** pod -> **Y** port.
5. **Robot Configuration (Driver Hub)**
   - Add the Pinpoint as a *goBILDA Pinpoint* device. Remember the exact name you give it.
   - Confirm the 4 drive motors are named exactly: `FrontLeft`, `FrontRight`, `BackLeft`, `BackRight`
     (case sensitive - these come from your teleop).
   - **If your Pinpoint is NOT named `pinpoint`**: change `c.name.set("pinpoint")` in
     `pedro/Constants.java`, and type the same name in AutoTune later.
6. Fresh battery, robot with its normal weight/mechanisms, **6+ ft of clear floor** forward and to the left.
7. Run **Build/Run** to deploy to the Control Hub.

## PART 3 - AutoTune (laptop + robot)
8. Connect the laptop Wi-Fi to the Control Hub network.
9. Browser: **http://192.168.43.1:10158**
   (The Pedro docs only say to open this address. If the page doesn't load, look for a Pedro/AutoTune
   OpMode in the Driver Hub list and run it - the docs don't mention one, so this is a fallback guess.)

### 3A. Mecanum Procedure
- Front Left = `FrontLeft`, Front Right = `FrontRight`, Back Left = `BackLeft`, Back Right = `BackRight` -> Submit.
- Continue. Each wheel spins one at a time; compare with the 3D diagram and answer correct/reversed.
- Open the **Java** tab -> copy the generated `drivetrainConfig`.

### 3B. Pinpoint Procedure
- HardwareMap Name: your Pinpoint name (placeholder: `pinpoint`).
- **Odometry Pod Type: choose SWING ARM** (the dropdown defaults to *Four Bar* - change it!).
- Continue -> push the robot straight **FORWARD**, then stop the OpMode.
- Continue -> push the robot to the **LEFT**, then stop the OpMode.
- Continue -> spin the robot **180 degrees COUNTER-clockwise** in place, then stop the OpMode.
- Java tab -> copy the generated `localizerConfig`.
- (If you see `ForceStopException`: you stopped before moving the robot. Just rerun.)

### 3C. Paste results into `pedro/Constants.java`
- Replace each block marked `1) DRIVETRAIN`, `2) LOCALIZER`, `3) FORESIGHT` with the generated one
  (from `public static ... =` through the closing `);`).
- For the drivetrain, keep the extra line `c.manualBrakeMode.set(true);` inside the config.
- Rebuild + redeploy after pasting the first two, then do the localization test:
  AutoTune > **Tests > Localization**. Start = (0,0); **forward -> x goes up; strafe left -> y goes up.**
  If not, rerun the Pinpoint Procedure before continuing.

### 3D. Foresight Procedure
Run in order (the robot drives itself - hands clear, drifting past the distance is normal):
1. Forward Velocity  2. Strafe Velocity  3. Forward Deceleration  4. Strafe Deceleration
5. Heading Braking  6. Heading Tuner  7. Forward Braking  8. Strafe Braking
9. Forward Translational  10. Strafe Translational
- Java tab -> copy `ForesightConfig` -> paste over block `3) FORESIGHT` (**this replaces my placeholder numbers**).
- Rebuild + redeploy.

### 3E. Verify (AutoTune > Tests)
- **Line Test** (48 in back and forth, should stay straight)
- **Curved Test** (smooth quarter circle)
- **Hold Test** (push the robot - it should return)
If a test is bad, rerun the matching Foresight step, not the whole thing.

---------------------------------------------------------------------------
## PART 4 - Using Pedro in your OpModes
`RedTeleOp` already creates the follower with `Constants.create(hardwareMap)`.
Paths from your visualizer: build them with the Pedro 3 path API
(https://pedropathing.com/docs/pathing/guide/path-creation) - send me the visualizer export and I'll convert it.

---------------------------------------------------------------------------
## Things to know / placeholders still in the code
| Where | What | Status |
|---|---|---|
| `Constants.localizerConfig` name | `"pinpoint"` | **PLACEHOLDER** - match your robot config |
| `Constants.localizerConfig` offsets | `0.0, 0.0` | **PLACEHOLDER** - Pinpoint Procedure replaces |
| `Constants.foresightConfig` | example numbers from the Pedro docs (another robot) | **PLACEHOLDER** - Foresight Procedure replaces. Do not drive paths on these |
| `RedTeleOp` start pose `(72,12,90deg)` | your existing TODO | unchanged, set to real start |
| `RedTeleOp` flywheel PID gains `0.001,0,0` | your existing TODO | unchanged |

## What changed in `RedTeleOp.java` (compare with `_pedro_setup_backup/...`)
- Pedro 2.x API -> 3.x: `Pose` now `com.pedropathing.math.Pose`; `follower.pose()`, `pose.x()/y()/heading()`;
  `follower.setPose(..)`; follower built by `Constants.create(hardwareMap)`.
- **Driving now goes through `follower.manual(y, -x, -rx)`.** In Pedro 3, `follower.update()` stops the drive
  motors when idle, so writing motor powers directly (the old way) would have fought it. Pedro's convention is
  strafe-left = positive and turn-CCW = positive, hence the sign flips. Driving feel/`TURN_SCALE` is the same.
  **Check on the robot that stick directions feel right** (forward = forward, right stick right = turn right).
- SDK 12 moved `id` off the base `AprilTagDetection`; added a `tagId()` helper. Cluster detections return -1
  (ignored) - revisit if the BIOBUZZ library gives you clusters for the cells.
- `PIDController` did not exist in your project; it now uses Pedro's (`calculate(target, error)`), flywheel gains
  are still your placeholder `0.001, 0, 0`.
- Not compiled/run by me - I have no Android toolchain here. Part 1 step 2 is the first real compile.
