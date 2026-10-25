package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.pedro.Constants;

/**
 * TurretTest - bare-bones test: drive + turn turret + move ramp + spin flywheel. Nothing else.
 *
 * CONTROLS
 *   Gamepad 1 left stick   : drive (forward / strafe)
 *   Gamepad 1 right stick  : turn
 *   Gamepad 2 right stick X: turn turret left / right
 *   Gamepad 2 left stick Y : (ramp - COMMENTED OUT for testing; search "RAMP COMMENTED OUT" to re-enable)
 *   Gamepad 2 A            : flywheel ON / OFF (toggle)
 *   Gamepad 2 X            : recenter turret and drop ramp to its lowest position
 *   Gamepad 2 B            : intake motor ON / OFF (toggle)
 *   Gamepad 2 RIGHT TRIGGER: hold ALL the way down = feeding motor + intake on; release = feeding stops
 *   Gamepad 2 D-pad up     : set flywheel power to 100% (press A to turn it on/off)
 *   Gamepad 2 D-pad down   : set flywheel power back to FLYWHEEL_POWER (-0.6)
 *
 * WIRING / HARDWARE CONFIG
 *   Drivetrain     : handled by Pedro (Constants.java) -> FrontLeft, FrontRight, BackLeft, BackRight
 *   Turret servo   : Expansion Hub servo port 1  -> config name "turretServo" <-- PLACEHOLDER name
 *   Ramp servo     : Expansion Hub servo port 0  -> config name "rampServo"   <-- PLACEHOLDER name
 *   Flywheel motor : config name "flywheel"                                    <-- PLACEHOLDER name
 *   Intake motor   : config name "intakemotor" (same name as IntakeTeleOp)
 *   The names are whatever you typed in the Driver Station "Configure Robot" screen
 *   for those ports. Change the three NAME lines below to match.
 */
@TeleOp(name = "Turret Test", group = "Test")
public class Turrettest extends OpMode {

    /* ---------------- HARDWARE CONFIG NAMES (PLACEHOLDERS) ---------------- */
    static final String TURRET_SERVO_NAME = "turretServo"; // Expansion Hub servo port 1 - TODO: confirm name
    static final String RAMP_SERVO_NAME   = "rampServo";   // Expansion Hub servo port 0 - TODO: confirm name
    static final String FLYWHEEL_NAME     = "flywheel";    // TODO: confirm name
    static final String INTAKE_MOTOR_NAME = "intakemotor"; // same name IntakeTeleOp already uses
    static final String FEEDING_MOTOR_NAME = "feedingmotor";

    /* ---------------- TURRET (PLACEHOLDERS) ---------------- */
    static final double TURRET_CENTER  = 0.5;   // TODO: servo position that points straight ahead (the starting direction)
    static final double TURRET_LIMIT_DEG = 150.0;       // turret may move this far left AND right of the starting direction
    static final double TURRET_RANGE_DEG = 300.0;       // TODO: full travel of the TURRET servo (goBILDA 2000 series = 300, REV smart servo = 270, standard = 180). If the turret is geared, use the turret degrees per full servo travel.
    static final double TURRET_LIMIT_POS = TURRET_LIMIT_DEG / TURRET_RANGE_DEG;                      // servo distance for 150 degrees
    static final double TURRET_MIN_POS = Math.max(0.0, TURRET_CENTER - TURRET_LIMIT_POS);            // 150 degrees to one side
    static final double TURRET_MAX_POS = Math.min(1.0, TURRET_CENTER + TURRET_LIMIT_POS);            // 150 degrees to the other side
    static final double TURRET_RATE    = 0.005; // how fast the stick moves the turret per loop (was 0.01, now 50% speed). Lower = slower.
    static final double TURRET_DEADZONE = 0.1;  // slight deadzone: ignore stick movement below this. Raise if it twitches at rest.
    static final double TURRET_DIRECTION = 1.0; // 1.0 = stick right turns turret right. If it goes the wrong way, set to -1.0

    /* ---------------- RAMP LIMIT ---------------- */
    // The ramp may only travel about 40 degrees total because the hardware can't go further.
    static final double SERVO_RANGE_DEG = 270.0; // TODO: full travel of YOUR servo (REV smart servo = 270, goBILDA 2000 series = 300, standard = 180)
    static final double RAMP_MAX_DEG    = 40.0;  // hard limit
    static final double RAMP_MIN_POS    = 0.0;   // TODO: servo position where the ramp is fully down
    static final double RAMP_MAX_POS    = RAMP_MIN_POS + (RAMP_MAX_DEG / SERVO_RANGE_DEG); // about 0.15 if range is 270
    // If the ramp moves the wrong way (goes DOWN when it should go UP and hits something),
    // set RAMP_MIN_POS to 1.0 and change RAMP_MAX_POS to RAMP_MIN_POS - (RAMP_MAX_DEG / SERVO_RANGE_DEG).
    static final double RAMP_RATE       = 0.005; // how fast the stick moves the ramp per loop

    /* ---------------- OTHER (PLACEHOLDERS) ---------------- */
    static final double STICK_DEADZONE = 0.1;
    static final double FLYWHEEL_POWER = -0.6;  // TODO: start low, raise slowly
    static final double TURN_SCALE     = 0.6;
    static final double FLYWHEEL_FULL_POWER = -1.0; // 100% power, same direction as FLYWHEEL_POWER
    static final double FEED_POWER     = -1.0;  // feeding motor power while the trigger is held
    static final double TRIGGER_THRESHOLD = 0.9; // right trigger must be pushed (almost) ALL the way down
    static final double INTAKE_POWER   = 1.0;   // same sign IntakeTeleOp uses; flip to 1.0 if it runs backwards

    /* ---------------- HARDWARE ---------------- */
    Follower follower;
    Servo turretServo;
    // Servo rampServo;   // RAMP COMMENTED OUT FOR TESTING
    DcMotorEx flywheel;
    DcMotor intakeMotor;
    DcMotor feedingMotor;

    /* ---------------- STATE ---------------- */
    double turretPos = TURRET_CENTER;
    double rampPos = RAMP_MIN_POS;
    boolean flywheelOn = false;
    double flywheelPower = FLYWHEEL_POWER; // current flywheel power, D-pad up = 100%, D-pad down = back to FLYWHEEL_POWER
    boolean flywheelToggleReady = true;
    boolean intakeOn = false;
    boolean intakeToggleReady = true;

    @Override
    public void init() {
        // Drivetrain through Pedro (uses Constants.java, same as RedTeleOp)
        follower = Constants.create(hardwareMap);
        follower.setPose(new Pose(0, 0, 0));

        // Turret + ramp servos
        turretServo = hardwareMap.get(Servo.class, TURRET_SERVO_NAME);
        // rampServo   = hardwareMap.get(Servo.class, RAMP_SERVO_NAME);   // RAMP COMMENTED OUT FOR TESTING
        // TODO: if the turret turns the wrong way, set to REVERSE
        turretServo.setDirection(Servo.Direction.FORWARD);
        // rampServo.setDirection(Servo.Direction.FORWARD);   // RAMP COMMENTED OUT
        turretServo.setPosition(TURRET_CENTER);
        // rampServo.setPosition(RAMP_MIN_POS);   // RAMP COMMENTED OUT

        // Flywheel
        flywheel = hardwareMap.get(DcMotorEx.class, FLYWHEEL_NAME);
        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        // TODO: if the flywheel spins the wrong way, uncomment:
        // flywheel.setDirection(DcMotor.Direction.REVERSE);
        flywheel.setPower(0);

        // Intake
        intakeMotor = hardwareMap.dcMotor.get(INTAKE_MOTOR_NAME);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setPower(0);

        // Feeding motor
        feedingMotor = hardwareMap.dcMotor.get(FEEDING_MOTOR_NAME);
        feedingMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        feedingMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        feedingMotor.setPower(0);

        telemetry.addLine("Turret Test ready.");
        telemetry.addLine("GP1: drive | GP2 R-stick: turret | GP2 L-stick: ramp | GP2 A: flywheel | GP2 B: intake | GP2 X: reset");
        telemetry.update();
    }

    @Override
    public void loop() {
        /* -------- DRIVE (gamepad1) -------- */
        double y  = -gamepad1.left_stick_y;
        double x  =  gamepad1.left_stick_x;
        double rx =  gamepad1.right_stick_x * TURN_SCALE;
        follower.manual(y, -x, -rx);   // same call RedTeleOp uses
        follower.update();

        /* -------- TURRET + RAMP -------- */
        if (gamepad2.x) {
            turretPos = TURRET_CENTER;
            rampPos = RAMP_MIN_POS;
        } else {
            // Stick direction = turret direction. Turret keeps turning while the stick is held off-center
            // and stops (holds position) the moment the stick returns to center. No preset target.
            // Deadzone, then rescale so speed starts from zero just outside the deadzone (smooth, not a jump).
            double raw = gamepad2.right_stick_x;
            double turn = Math.abs(raw) > TURRET_DEADZONE
                    ? Math.signum(raw) * (Math.abs(raw) - TURRET_DEADZONE) / (1.0 - TURRET_DEADZONE)
                    : 0;
            turretPos = Range.clip(turretPos + TURRET_DIRECTION * turn * TURRET_RATE, TURRET_MIN_POS, TURRET_MAX_POS);

            // RAMP COMMENTED OUT FOR TESTING
            // double lift = Math.abs(gamepad2.left_stick_y) > STICK_DEADZONE ? -gamepad2.left_stick_y : 0;
            // rampPos = rampPos + lift * RAMP_RATE;
        }
        // Hard clamp so the ramp can never be commanded past the 40 degree limit
        // rampPos = Range.clip(rampPos, Math.min(RAMP_MIN_POS, RAMP_MAX_POS), Math.max(RAMP_MIN_POS, RAMP_MAX_POS));   // RAMP COMMENTED OUT

        turretServo.setPosition(turretPos);
        // rampServo.setPosition(rampPos);   // RAMP COMMENTED OUT

        /* -------- FLYWHEEL (gamepad2 A toggles) -------- */
        if (!gamepad2.a) flywheelToggleReady = true;
        if (gamepad2.a && flywheelToggleReady) {
            flywheelOn = !flywheelOn;
            flywheelToggleReady = false;
        }
        if (gamepad2.dpad_up) flywheelPower = FLYWHEEL_FULL_POWER; // 100%
        if (gamepad2.dpad_down) flywheelPower = FLYWHEEL_POWER;    // back to normal (-0.6)
        flywheel.setPower(flywheelOn ? flywheelPower : 0);

        /* -------- INTAKE (gamepad2 B toggles) -------- */
        if (!gamepad2.b) intakeToggleReady = true;
        if (gamepad2.b && intakeToggleReady) {
            intakeOn = !intakeOn;
            intakeToggleReady = false;
        }
        /* -------- FEEDING MOTOR (gamepad2 RIGHT TRIGGER, hold all the way down) -------- */
        // While the trigger is pushed past TRIGGER_THRESHOLD: feeding motor AND intake run at full power.
        // Let go and the feeding motor stops; the intake goes back to whatever the B toggle says.
        boolean feeding = gamepad2.right_trigger > TRIGGER_THRESHOLD;
        feedingMotor.setPower(feeding ? FEED_POWER : 0);
        intakeMotor.setPower((intakeOn || feeding) ? INTAKE_POWER : 0);

        /* -------- TELEMETRY -------- */
        telemetry.addData("Turret pos", "%.3f (about %.0f deg from start, limit +/-%.0f)", turretPos,
                (turretPos - TURRET_CENTER) * TURRET_RANGE_DEG, TURRET_LIMIT_DEG);
        // telemetry.addData("Ramp pos", "%.3f (about %.0f deg of travel)", rampPos,   // RAMP COMMENTED OUT
        //         Math.abs(rampPos - RAMP_MIN_POS) * SERVO_RANGE_DEG);
        telemetry.addData("Flywheel", flywheelOn ? "ON" : "OFF");
        telemetry.addData("Flywheel power", "%.0f%%", Math.abs(flywheelPower) * 100);
        telemetry.addData("Intake", (intakeOn || feeding) ? "ON" : "OFF");
        telemetry.addData("Feeding motor", feeding ? "ON" : "OFF");
        telemetry.addData("Flywheel velocity (ticks/s)", "%.0f", flywheel.getVelocity());
        telemetry.update();
    }

    @Override
    public void stop() {
        if (flywheel != null) flywheel.setPower(0);
        if (intakeMotor != null) intakeMotor.setPower(0);
        if (feedingMotor != null) feedingMotor.setPower(0);
        if (follower != null) {
            follower.stop();
            follower.update();
        }
    }
}
