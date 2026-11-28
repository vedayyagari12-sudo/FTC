package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.RobotConfig.*;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import org.firstinspires.ftc.teamcode.pedro.Constants;

/**
 * Red TeleOp - drive, intake and a fixed shooter (no turret), with the same flywheel controls as Turret Test.
 * BlueTeleOp extends this class; only the HIVE used for the distance readout differs.
 *
 * CONTROLS
 *   Gamepad 1 left stick    : drive (forward / strafe)
 *   Gamepad 1 right stick   : turn
 *   Gamepad 2 A             : flywheel ON / OFF (toggle)
 *   Gamepad 2 D-pad up/down : flywheel speed +5% / -5% (percent of FLYWHEEL_MAX_RPM, held by FlywheelPIDF)
 *   Gamepad 2 D-pad right/left: flywheel speed +1% / -1%
 *   Gamepad 2 B             : intake ON / OFF (toggle)
 *   Gamepad 2 RIGHT TRIGGER : hold ALL the way down = feeding motor + intake on (shoot)
 *   Gamepad 2 left stick Y  : move the ramp (same code and limits as Turret Test: Turrettest.RAMP_*)
 *   Gamepad 2 X             : drop the ramp to its lowest position
 *
 * Odometry still runs: telemetry shows the robot pose and its distance to the CELL for checking against a tape.
 * Hardware names, powers, PIDF gains and the start pose are in RobotConfig.java.
 */
@TeleOp(name = "Red TeleOp")
public class RedTeleOp extends OpMode {

    // Same steps and start as Turret Test, but starting at 45% = 2700 RPM, the speed tested at 32 in.
    static final int START_PERCENT = 45;
    static final int SMALL_STEP_PERCENT = 1;
    static final int BIG_STEP_PERCENT = 5;
    static final double TRIGGER_THRESHOLD = 0.9; // right trigger must be pushed (almost) ALL the way down

    /* ---------------- HARDWARE ---------------- */
    DcMotor IntakeMotor; // drive wheels are driven by Pedro 3's Follower (see loop())
    CRServo IntakeServo;
    DcMotorEx flywheel;
    DcMotor feedingMotor;
    Servo rampServo;
    Follower follower;
    FlywheelPIDF flywheelPID; // same controller as Turret Test; gains FLYWHEEL_P/I/D/F in RobotConfig

    // Which alliance HIVE the distance readout uses. BlueTeleOp returns TurretAim.BLUE.
    int alliance() { return TurretAim.RED; }

    /* ---------------- STATE ---------------- */
    boolean intakeOn = false;
    boolean intakeToggleReady = true;
    boolean flywheelOn = false;
    boolean flywheelToggleReady = true;
    int flywheelPercent = START_PERCENT;
    boolean dpadReady = true;
    double rampPos = Turrettest.RAMP_MIN_POS;

    /* ---------------- INIT ---------------- */
    @Override
    public void init() {
        IntakeMotor  = hardwareMap.dcMotor.get(INTAKE_MOTOR_NAME);
        IntakeServo  = hardwareMap.get(CRServo.class, INTAKE_SERVO_NAME);
        IntakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        IntakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        stopIntake();

        flywheel = hardwareMap.get(DcMotorEx.class, FLYWHEEL_NAME);
        // Reverse so the shooting direction is positive power and positive RPM.
        flywheel.setDirection(FLYWHEEL_DIRECTION < 0 ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);
        // The autos used to leave the flywheel in RUN_USING_ENCODER; the hub's own speed loop would fight FlywheelPIDF.
        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheel.setPower(0);
        flywheelPID = new FlywheelPIDF(flywheel);

        feedingMotor = hardwareMap.dcMotor.get(FEEDING_MOTOR_NAME);
        feedingMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        feedingMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        feedingMotor.setPower(0);

        // Ramp: same setup as Turret Test.
        rampServo = hardwareMap.get(Servo.class, RAMP_SERVO_NAME);
        rampServo.setDirection(Servo.Direction.FORWARD);
        rampServo.setPosition(Turrettest.RAMP_MIN_POS);

        follower = Constants.create(hardwareMap); // Pedro 3
        // Start where the auto ended (if one just ran), otherwise at TELEOP_START_*. Clear it so a second
        // TeleOp run without an auto doesn't reuse an old pose.
        boolean fromAuto = USE_AUTO_END_POSE && autoEndPose != null;
        Pose start = fromAuto ? autoEndPose : new Pose(TELEOP_START_X, TELEOP_START_Y, Math.toRadians(TELEOP_START_HEADING_DEG));
        autoEndPose = null;
        follower.setPose(start);
        telemetry.addData("Start pose", "%s: x %.1f  y %.1f  heading %.0f deg", fromAuto ? "from auto" : "default start",
                start.x(), start.y(), Math.toDegrees(start.heading()));

        telemetry.addLine((alliance() == TurretAim.RED ? "Red" : "Blue") + " TeleOp initialized. GP2: A flywheel, D-pad speed, B intake, RT shoot.");
        telemetry.update();
    }

    /* ---------------- LOOP ---------------- */
    @Override
    public void loop() {

        /* -------- MECANUM DRIVE (gamepad1) -------- */
        double y  = -gamepad1.left_stick_y;
        double x  =  gamepad1.left_stick_x;
        double rx =  gamepad1.right_stick_x * TURN_SCALE;

        // Pedro 3: positive strafe = LEFT and positive turn = counter-clockwise, so the stick
        // axes are inverted. follower.update() applies these powers.
        follower.manual(y, -x, -rx);
        follower.update();

        /* -------- FLYWHEEL (gamepad2 A toggles, D-pad sets speed) -------- */
        if (!gamepad2.a) flywheelToggleReady = true;
        if (gamepad2.a && flywheelToggleReady) {
            flywheelOn = !flywheelOn;
            flywheelToggleReady = false;
        }
        // One step per press: right/left = 1%, up/down = 5%. Holding a button does not repeat.
        boolean dpadHeld = gamepad2.dpad_up || gamepad2.dpad_down || gamepad2.dpad_left || gamepad2.dpad_right;
        if (!dpadHeld) dpadReady = true;
        if (dpadHeld && dpadReady) {
            int step = gamepad2.dpad_up ? BIG_STEP_PERCENT : gamepad2.dpad_down ? -BIG_STEP_PERCENT
                     : gamepad2.dpad_right ? SMALL_STEP_PERCENT : -SMALL_STEP_PERCENT;
            flywheelPercent = Math.max(0, Math.min(100, flywheelPercent + step));
            dpadReady = false;
        }
        double targetRpm = flywheelPercent / 100.0 * FLYWHEEL_MAX_RPM;
        double currentRpm = flywheelPID.rpm();
        flywheel.setPower(flywheelPID.pidf(flywheelOn ? targetRpm : 0));
        boolean atSpeed = flywheelOn && Math.abs(targetRpm - currentRpm) < FLYWHEEL_RPM_TOLERANCE;

        /* -------- INTAKE (gamepad2 B toggles) + SHOOT (gamepad2 RIGHT TRIGGER) -------- */
        if (!gamepad2.b) intakeToggleReady = true;
        if (gamepad2.b && intakeToggleReady) {
            intakeOn = !intakeOn;
            intakeToggleReady = false;
        }
        // Same as Turret Test: trigger all the way down runs the feeding motor AND the intake.
        boolean feeding = gamepad2.right_trigger > TRIGGER_THRESHOLD;
        feedingMotor.setPower(feeding ? FEED_POWER : 0);
        if (intakeOn || feeding) runIntake(); else stopIntake();

        /* -------- RAMP (gamepad2 left stick Y moves it, X drops it): same as Turret Test -------- */
        if (gamepad2.x) {
            rampPos = Turrettest.RAMP_MIN_POS;
        } else {
            double lift = Math.abs(gamepad2.left_stick_y) > STICK_DEADZONE ? -gamepad2.left_stick_y : 0;
            rampPos = rampPos + lift * Turrettest.RAMP_RATE;
        }
        // Hard clamp so the ramp can never be commanded past the 40 degree limit
        rampPos = Range.clip(rampPos, Math.min(Turrettest.RAMP_MIN_POS, Turrettest.RAMP_MAX_POS),
                Math.max(Turrettest.RAMP_MIN_POS, Turrettest.RAMP_MAX_POS));
        rampServo.setPosition(rampPos);

        /* -------- TELEMETRY -------- */
        Pose pose = follower.pose();
        TurretAim.Result aim = TurretAim.aim(pose.x(), pose.y(), pose.heading(), alliance());
        telemetry.addData("Flywheel", flywheelOn ? "ON" : "OFF");
        telemetry.addData("Flywheel speed", "%d%% = %.0f RPM target, %.0f actual%s", flywheelPercent, targetRpm, currentRpm,
                atSpeed ? "  AT SPEED" : "");
        telemetry.addData("Intake", (intakeOn || feeding) ? "ON" : "OFF");
        telemetry.addData("Feeding motor", feeding ? "ON" : "OFF");
        telemetry.addData("Ramp pos", "%.3f (about %.0f deg of travel)", rampPos,
                Math.abs(rampPos - Turrettest.RAMP_MIN_POS) * Turrettest.SERVO_RANGE_DEG);
        // For checking odometry against a tape measure; the RPM does not use it.
        telemetry.addData("Distance", "%.1f in (flywheel shaft to %s CELL)", aim.distanceIn, aim.targetFar ? "far" : "audience");
        telemetry.addData("Robot pose", "x %.1f  y %.1f  heading %.0f deg", pose.x(), pose.y(), Math.toDegrees(pose.heading()));
        telemetry.update();
    }

    /* ---------------- INTAKE HELPERS ---------------- */
    private void runIntake() {
        IntakeMotor.setPower(INTAKE_MOTOR_POWER);
        IntakeServo.setPower(INTAKE_SERVO_POWER);
    }

    private void stopIntake() {
        IntakeMotor.setPower(0);
        IntakeServo.setPower(0);
    }

    @Override
    public void stop() {
        stopIntake();
        if (feedingMotor != null) feedingMotor.setPower(0);
        if (flywheel != null) flywheel.setPower(0);
        if (follower != null) { follower.stop(); follower.update(); } // zeroes the drive motors
    }
}
