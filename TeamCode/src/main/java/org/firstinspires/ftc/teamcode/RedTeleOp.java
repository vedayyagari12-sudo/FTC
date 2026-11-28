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

@TeleOp(name = "Red TeleOp")
public class RedTeleOp extends OpMode {

    private RampControl ramp; // flower ramp: lowered to the floor at start (positions in RobotConfig)

    /* Hardware names, powers, flywheel numbers, shot table and start pose are in RobotConfig.java. */
    // No hood servo in the config. rampServo (Expansion Hub servo 0) is probably the hood, but its safe
    // positions are not measured yet (see Turrettest), so this TeleOp does not drive it.

    /* ---------------- HARDWARE ---------------- */
    DcMotor IntakeMotor; // drive wheels are driven by Pedro 3's Follower (see loop())
    CRServo IntakeServo;

    Servo turretServo;
    DcMotorEx flywheel;
    DcMotor feedingMotor;
    Follower follower;
    FlywheelPIDF flywheelPID; // same controller as Turret Test; gains FLYWHEEL_P/I/D/F in RobotConfig

    /* ---------------- HIVE CELL CONFIG ---------------- */
    // Which alliance HIVE to aim at. BlueTeleOp extends this class and returns TurretAim.BLUE.
    int alliance() { return TurretAim.RED; }

    // "Back" cell = TurretAim's far CELL. Cell positions, turret mapping and limits all come from TurretAim.java.

    // No camera: the turret aims from the Pinpoint pose alone. The CELL is picked by the half line (below it TurretAim
    // aims at the audience CELL, above it the far/back CELL) unless gamepad 2 D-pad forces one.
    final double FIELD_HALF_DIVIDER = 72;

    /* ---------------- TURRET CONFIG ---------------- */
    // Servo positions for TurretAim's turn limit to each side (manual mode stays inside these).
    final double TURRET_MIN_POS = Math.max(0.0, TurretAim.TURRET_CENTER - TurretAim.TURRET_LIMIT_DEG / TurretAim.TURRET_RANGE_DEG);
    final double TURRET_MAX_POS = Math.min(1.0, TurretAim.TURRET_CENTER + TurretAim.TURRET_LIMIT_DEG / TurretAim.TURRET_RANGE_DEG);
    final double TURRET_TOLERANCE_DEG = 2.0;
    final long TURRET_SETTLE_MS = 150;

    /* ---------------- MANUAL MODE CONFIG ---------------- */
    final double MANUAL_TURRET_RATE = 0.01;

    /* ---------------- STATE ---------------- */
    boolean intakeOn = false;
    boolean intakeToggleReady = true;

    boolean targetIsBackCell = true;
    Boolean cellOverride = null; // null = pick by half line; true = far/back CELL; false = audience CELL
    double lastCommandedAngle = Double.NaN;
    long lastAngleChangeTime = 0;
    boolean flywheelAtSpeed = false;

    boolean manualMode = false;
    boolean modeToggleReady = true;
    double manualTurretPos = TurretAim.TURRET_CENTER;
    double lastTurretPos = TurretAim.TURRET_CENTER;

    /* ---------------- INIT ---------------- */
    @Override
    public void init() {
        IntakeMotor  = hardwareMap.dcMotor.get(INTAKE_MOTOR_NAME);
        IntakeServo  = hardwareMap.get(CRServo.class, INTAKE_SERVO_NAME);

        IntakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        IntakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        stopIntake();

        turretServo  = hardwareMap.get(Servo.class, TURRET_SERVO_NAME);
        flywheel     = hardwareMap.get(DcMotorEx.class, FLYWHEEL_NAME);
        // The autos spin the flywheel with FLYWHEEL_DIRECTION * speed; reversing the motor here gives the same
        // direction with positive power, so the RPM math below stays positive.
        flywheel.setDirection(FLYWHEEL_DIRECTION < 0 ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);
        // The autos leave the flywheel in RUN_USING_ENCODER; the hub's own speed loop would fight FlywheelPIDF.
        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        feedingMotor = hardwareMap.dcMotor.get(FEEDING_MOTOR_NAME);
        feedingMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        feedingMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        feedingMotor.setPower(0);
        flywheelPID = new FlywheelPIDF(flywheel);

        follower = Constants.create(hardwareMap); // Pedro 3
        // Start where the auto ended (if one just ran), otherwise at TELEOP_START_*. Clear it so a second
        // TeleOp run without an auto doesn't reuse an old pose.
        boolean fromAuto = USE_AUTO_END_POSE && autoEndPose != null;
        Pose start = fromAuto ? autoEndPose : new Pose(TELEOP_START_X, TELEOP_START_Y, Math.toRadians(TELEOP_START_HEADING_DEG));
        autoEndPose = null;
        follower.setPose(start);
        ramp = new RampControl(hardwareMap); // does not move the ramp yet
        telemetry.addData("Start pose", "%s: x %.1f  y %.1f  heading %.0f deg", fromAuto ? "from auto" : "default start",
                start.x(), start.y(), Math.toDegrees(start.heading()));

        telemetry.addLine((alliance() == TurretAim.RED ? "Red" : "Blue") + " TeleOp initialized. GP2: B intake, LB auto/manual, D-pad up/down/left force far/audience/auto cell, RT fire.");
        telemetry.update();
    }

    /* ---------------- LOOP ---------------- */
    @Override
    public void loop() {
        // Flower ramp: HOLD gamepad 2 left trigger to put it on the floor; let go and it goes back to its default (up) position.
        if (gamepad2.left_trigger > RAMP_TRIGGER) ramp.lower(); else ramp.raise();
        ramp.update();

        /* -------- MECANUM DRIVE (gamepad1) -------- */
        double y  = -gamepad1.left_stick_y;
        double x  =  gamepad1.left_stick_x;
        double rx =  gamepad1.right_stick_x * TURN_SCALE;

        // Pedro 3: positive strafe = LEFT and positive turn = counter-clockwise, so the stick
        // axes are inverted. follower.update() (called later in this loop) applies these powers.
        follower.manual(y, -x, -rx);

        /* -------- INTAKE TOGGLE (gamepad2 B) -------- */
        if (!gamepad2.b) intakeToggleReady = true;
        if (gamepad2.b && intakeToggleReady) {
            intakeOn = !intakeOn;
            intakeToggleReady = false;
        }
        if (gamepad2.x) reverseIntake();        // HOLD gamepad 2 X = run the intake backwards (clears a jam)
        else if (intakeOn) runIntake();
        else stopIntake();

        /* -------- AUTO / MANUAL TOGGLE (gamepad2 LB) -------- */
        if (!gamepad2.left_bumper) modeToggleReady = true;
        if (gamepad2.left_bumper && modeToggleReady) {
            manualMode = !manualMode;
            modeToggleReady = false;
            // Start manual from wherever auto left the turret so nothing jumps.
            manualTurretPos = lastTurretPos;
        }

        /* -------- TURRET / SHOOTER -------- */
        updateTurretAndShooter();

        // Fire on gamepad2 RT in both modes. Auto waits for aim + flywheel,
        // manual only waits for flywheel since the driver is aiming.
        boolean readyToFire = manualMode ? flywheelAtSpeed : turretReady();
        feedingMotor.setPower(gamepad2.right_trigger > 0.5 && readyToFire ? FEED_POWER : 0);

        telemetry.addData("Aim mode", manualMode ? "MANUAL" : "AUTO");
        telemetry.addData("Ready to fire", readyToFire);

        telemetry.addData("Intake", gamepad2.x ? "REVERSING" : (intakeOn ? "RUNNING" : "STOPPED"));
        telemetry.addData("Ramp", ramp.status());
        telemetry.update();
    }

    /* ---------------- TURRET / SHOOTER LOGIC ---------------- */
    private void updateTurretAndShooter() {
        follower.update();
        Pose pose = follower.pose();

        updateHiveCellTarget(pose);

        TurretAim.Result aim = TurretAim.aim(pose.x(), pose.y(), pose.heading(), alliance(), targetIsBackCell);
        double turretAngle = aim.turretDeg;
        double servoPos = aim.servoPos;

        if (manualMode) {
            double turn = Math.abs(gamepad2.right_stick_x) > STICK_DEADZONE ? gamepad2.right_stick_x : 0;
            manualTurretPos = Range.clip(manualTurretPos + turn * MANUAL_TURRET_RATE, TURRET_MIN_POS, TURRET_MAX_POS);
            servoPos = manualTurretPos;
        }
        turretServo.setPosition(servoPos);
        lastTurretPos = servoPos;

        if (Double.isNaN(lastCommandedAngle) || Math.abs(turretAngle - lastCommandedAngle) > TURRET_TOLERANCE_DEG) {
            lastAngleChangeTime = System.currentTimeMillis();
        }
        lastCommandedAngle = turretAngle;

        double distance = aim.distanceIn;
        updateShooterForDistance(distance);

        telemetry.addData("Hive target", targetIsBackCell ? "BACK" : "AUDIENCE");
        telemetry.addData("Turret", "%.1f deg, servo %.3f%s", aim.turretDeg, servoPos, aim.outOfRange ? " (OUT OF RANGE)" : "");
        telemetry.addData("Distance", "%.1f in (flywheel shaft to CELL)", distance);
        // For checking odometry against a tape measure: where the robot thinks it is, and the CELL point it aims at.
        telemetry.addData("Robot pose", "x %.1f  y %.1f  heading %.0f deg", pose.x(), pose.y(), Math.toDegrees(pose.heading()));
        telemetry.addData("CELL point", "x %.1f  y %.1f", aim.targetX, aim.targetY);
    }

    // D-pad up = far (back) CELL, down = audience CELL, left = back to automatic (half line).
    private void updateHiveCellTarget(Pose pose) {
        if (gamepad2.dpad_up)   cellOverride = true;
        if (gamepad2.dpad_down) cellOverride = false;
        if (gamepad2.dpad_left) cellOverride = null;
        targetIsBackCell = cellOverride != null ? cellOverride : pose.y() >= FIELD_HALF_DIVIDER;
    }

    private boolean turretReady() {
        return System.currentTimeMillis() - lastAngleChangeTime > TURRET_SETTLE_MS && flywheelAtSpeed;
    }

    // Blends between the nearest measured distance/RPM points in the table.
    private void updateShooterForDistance(double distance) {
        double targetRPM = shotRpm(distance);

        double currentRPM = flywheelPID.rpm();
        flywheel.setPower(flywheelPID.pidf(targetRPM));
        flywheelAtSpeed = Math.abs(targetRPM - currentRPM) < FLYWHEEL_RPM_TOLERANCE;

        telemetry.addData("Flywheel RPM", "%.0f target, %.0f actual", targetRPM, currentRPM);
    }

    /* ---------------- INTAKE HELPERS ---------------- */
    private void runIntake() {
        IntakeMotor.setPower(INTAKE_MOTOR_POWER);
        IntakeServo.setPower(INTAKE_SERVO_POWER);
    }

    private void reverseIntake() {
        IntakeMotor.setPower(-INTAKE_MOTOR_POWER);
        IntakeServo.setPower(-INTAKE_SERVO_POWER);
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
