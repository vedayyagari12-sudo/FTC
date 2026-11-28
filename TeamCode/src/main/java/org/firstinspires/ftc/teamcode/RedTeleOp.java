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
    // Driver-picked RPM: gamepad2 D-pad up/down steps through the SHOT_TABLE rows (odometry distance is
    // not trusted yet). Starts on the row nearest the 32 in tested shot.
    int presetIndex = nearestShotRow(32);
    boolean presetDpadReady = true;
    double lastCommandedAngle = Double.NaN;
    long lastAngleChangeTime = 0;
    boolean flywheelAtSpeed = false;

    boolean manualMode = true; // turret removed: MANUAL fires once the flywheel is at speed (AUTO also waits for the turret)
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
        telemetry.addData("Start pose", "%s: x %.1f  y %.1f  heading %.0f deg", fromAuto ? "from auto" : "default start",
                start.x(), start.y(), Math.toDegrees(start.heading()));

        telemetry.addLine((alliance() == TurretAim.RED ? "Red" : "Blue") + " TeleOp initialized. GP2: B intake, LB auto/manual aim, D-pad up/down shot preset, RT fire.");
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
        // axes are inverted. follower.update() (called later in this loop) applies these powers.
        follower.manual(y, -x, -rx);

        /* -------- INTAKE TOGGLE (gamepad2 B) -------- */
        if (!gamepad2.b) intakeToggleReady = true;
        if (gamepad2.b && intakeToggleReady) {
            intakeOn = !intakeOn;
            intakeToggleReady = false;
        }
        if (intakeOn) runIntake(); else stopIntake();

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

        telemetry.addData("Intake", intakeOn ? "RUNNING" : "STOPPED");
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

    // Far (back) CELL above the half line, audience CELL below it. (The D-pad now picks the shot preset.)
    private void updateHiveCellTarget(Pose pose) {
        targetIsBackCell = pose.y() >= FIELD_HALF_DIVIDER;
    }

    private boolean turretReady() {
        return System.currentTimeMillis() - lastAngleChangeTime > TURRET_SETTLE_MS && flywheelAtSpeed;
    }

    // RPM from the driver's preset (a SHOT_TABLE row). D-pad up = next farther row, down = next closer row.
    // To go back to odometry later: targetRPM = shotRpm(distance).
    private void updateShooterForDistance(double distance) {
        boolean dpad = gamepad2.dpad_up || gamepad2.dpad_down;
        if (dpad && presetDpadReady) presetIndex = stepPreset(presetIndex, gamepad2.dpad_up ? 1 : -1);
        presetDpadReady = !dpad;
        double targetRPM = SHOT_TABLE[presetIndex][1];
        telemetry.addData("Shot preset", "%.0f in, %.0f RPM (%d of %d, D-pad up/down)",
                SHOT_TABLE[presetIndex][0], targetRPM, presetIndex + 1, SHOT_TABLE.length);

        double currentRPM = flywheelPID.rpm();
        flywheel.setPower(flywheelPID.pidf(targetRPM));
        flywheelAtSpeed = Math.abs(targetRPM - currentRPM) < FLYWHEEL_RPM_TOLERANCE;

        telemetry.addData("Flywheel RPM", "%.0f target, %.0f actual", targetRPM, currentRPM);
    }

    /** Index of the SHOT_TABLE row whose distance is closest to distanceIn. */
    static int nearestShotRow(double distanceIn) {
        int best = 0;
        for (int i = 1; i < SHOT_TABLE.length; i++) {
            if (Math.abs(SHOT_TABLE[i][0] - distanceIn) < Math.abs(SHOT_TABLE[best][0] - distanceIn)) best = i;
        }
        return best;
    }

    /** Moves the preset by step rows, staying inside the table. */
    static int stepPreset(int index, int step) {
        return Math.max(0, Math.min(SHOT_TABLE.length - 1, index + step));
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
