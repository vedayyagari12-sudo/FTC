# Hardware config: `26season`

The robot configuration that is active on the Driver Station, copied from the "Active Configuration" screen (photos in [`doc/hardware/`](doc/hardware/)). Code must use these names exactly: they are case-sensitive, and a name that isn't in the config makes `hardwareMap.get(...)` throw at INIT.

## Control Hub

| Kind | Port | Name | Type |
|---|---|---|---|
| Motor | 0 | `BackLeft` | goBILDA 5202/3/4 series |
| Motor | 1 | `BackRight` | goBILDA 5202/3/4 series |
| Motor | 2 | `FrontLeft` | goBILDA 5202/3/4 series |
| Motor | 3 | `FrontRight` | goBILDA 5202/3/4 series |
| Servo | 0 | `Intakeservo` | Continuous Rotation Servo |
| I2C bus 0 | 0 | `imu` | REV internal IMU (BHI260AP) |
| I2C bus 1 | 0 | `pinpoint` | goBILDA Pinpoint Odometry Computer |

## Expansion Hub 2

| Kind | Port | Name | Type |
|---|---|---|---|
| Motor | 0 | `flywheel` | goBILDA 5202/3/4 series |
| Motor | 1 | `intakemotor` | goBILDA 5202/3/4 series |
| Motor | 2 | `feedingmotor` | goBILDA 5202/3/4 series |
| Servo | 0 | `rampServo` | Servo |
| Servo | 1 | `turretServo` | Servo |

## USB

| Name | Type |
|---|---|
| `Webcam` | Webcam. The photo cuts off below this line, so confirm the exact name on the Driver Station. |

## Who uses what

| Name | Used by |
|---|---|
| `FrontLeft`, `FrontRight`, `BackLeft`, `BackRight` | `pedro/Constants.java` (every OpMode that uses Pedro), `IntakeTeleOp` |
| `pinpoint` | `pedro/Constants.java` |
| `flywheel` | the four full autos, `Turrettest`, `RedTeleOp` |
| `intakemotor` | the four full autos, `Turrettest`, `RedTeleOp`, `IntakeTeleOp` |
| `feedingmotor` | the four full autos, `Turrettest` |
| `turretServo` | the four full autos (`TurretAim`), `Turrettest` |
| `Intakeservo` | the four full autos (runs with `intakemotor`) |
| `rampServo` | `Turrettest` only, and commented out there. The autos leave it alone until its down/up positions are measured. |
| `imu` | nothing yet (heading comes from the Pinpoint) |

## OpModes that do NOT match this config

These ask for names that are not in `26season`, so they will crash at INIT on this robot until they are updated:

- `RedTeleOp`: `intakeservo`, `intakeservo2`, `turretServo1`, `turretServo2`, `hoodServo`, `Webcam 1`
- `IntakeTeleOp`: `intakeservo`, `intakeservo2`, `shootmotor`, `intakeservo3`

## Photos

![Expansion Hub 2 and the top of the Control Hub](doc/hardware/26season-expansion-hub.png)

![Control Hub motors, servo, I2C and webcam](doc/hardware/26season-control-hub.png)
