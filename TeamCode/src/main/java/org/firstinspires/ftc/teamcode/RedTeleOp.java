package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import com.pedropathing.controllers.PIDController;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

@TeleOp(name = "Red TeleOp")
public class RedTeleOp extends OpMode {

    /* ---------------- HARDWARE MAP NAMES (update to match your config) ---------------- */
    // NOTE: the four drive-motor names below are now used by pedro/Constants.java (drivetrainConfig).
    // Pedro 3 owns the drive motors; if you rename them in the robot config, change them there too.
    final String FRONT_LEFT_NAME    = "FrontLeft";
    final String FRONT_RIGHT_NAME   = "FrontRight";
    final String BACK_LEFT_NAME     = "BackLeft";
    final String BACK_RIGHT_NAME    = "BackRight";
    final String INTAKE_MOTOR_NAME  = "intakemotor";
    // All names below match the 26season config (see HARDWARE_CONFIG.md).
    final String INTAKE_SERVO_NAME  = "Intakeservo";  // CR servo, Control Hub servo 0 (capital I)
    final String TURRET_SERVO_NAME  = "turretServo";  // the robot has ONE turret servo, Expansion Hub servo 1
    final String FLYWHEEL_NAME      = "flywheel";
    final String FEEDING_MOTOR_NAME = "feedingmotor"; // pushes balls into the flywheel
    final String WEBCAM_NAME        = "Webcam";       // TODO confirm: the config photo cuts off right at this line
    // No hood servo in the config. rampServo (Expansion Hub servo 0) is probably the hood, but its safe
    // positions are not measured yet (see Turrettest), so this TeleOp does not drive it.

    /* ---------------- HARDWARE ---------------- */
    DcMotor IntakeMotor; // drive wheels are driven by Pedro 3's Follower (see loop())
    CRServo IntakeServo;

    Servo turretServo;
    DcMotorEx flywheel;
    DcMotor feedingMotor;
    Follower follower;
    AprilTagProcessor aprilTag;
    VisionPortal visionPortal;
    PIDController flywheelPID; // TODO: swap in your actual PID class if the name/signature differs

    /* ---------------- DRIVE / INTAKE TUNING ---------------- */
    final double INTAKE_MOTOR_POWER  = -1.0;
    final double INTAKE_SERVO_POWER  = -1.0; // same sign as the autos and IntakeTeleOp; flip if it pushes balls out
    final double FEED_POWER          = -1.0; // same as Turrettest and the autos
    final double TURN_SCALE = 0.6;

    /* ---------------- HIVE / TAG CONFIG (red only) ---------------- */
    // Each cell's AprilTag faces down toward the tiles only when that cell's
    // opening faces up - so seeing a cell's tags means aim at THAT cell.
    final int[] BACK_CELL_TAGS     = {0, 1, 2, 3};
    final int[] AUDIENCE_CELL_TAGS = {4, 5, 6, 7};

    // "Back" cell = TurretAim's far CELL. Cell positions, turret mapping and limits all come from TurretAim.java.

    final long VISION_STALE_MS = 1000;
    final long MANUAL_OVERRIDE_GRACE_MS = 1000;

    // Half line: below it TurretAim aims at the audience CELL, above it the far (back) CELL.
    final double FIELD_HALF_DIVIDER = 72;

    /* ---------------- TURRET CONFIG ---------------- */
    // Servo positions for TurretAim's turn limit to each side (manual mode and vision trim stay inside these).
    final double TURRET_MIN_POS = Math.max(0.0, TurretAim.TURRET_CENTER - TurretAim.TURRET_LIMIT_DEG / TurretAim.TURRET_RANGE_DEG);
    final double TURRET_MAX_POS = Math.min(1.0, TurretAim.TURRET_CENTER + TurretAim.TURRET_LIMIT_DEG / TurretAim.TURRET_RANGE_DEG);
    final double TURRET_TOLERANCE_DEG = 2.0;
    final long TURRET_SETTLE_MS = 150;

    // Fine vision trim - only applied past FIELD_HALF_DIVIDER, on top of the
    // odometry aim, when a hive tag is actually visible.
    final double VISION_TRIM_GAIN = 0.02;
    final double MAX_VISION_TRIM_PER_LOOP = 0.01;

    /* ---------------- MANUAL MODE CONFIG ---------------- */
    final double MANUAL_TURRET_RATE = 0.01;
    final double STICK_DEADZONE = 0.1;

    /* ---------------- SHOOTER CONFIG ---------------- */
    // {distance in, RPM, hood pos} - hood pos is not used until the ramp is wired in. PLACEHOLDER rows, replace with tested values.
    final double[][] shotTable = {
        {24, 2400, 0.30},
        {60, 2900, 0.45},
        {96, 3400, 0.60},
    };
    final double FLYWHEEL_RPM_TOLERANCE = 100;
    final double FLYWHEEL_TICKS_PER_REV = 28.0; // TODO: set to your actual motor's ticks/rev

    /* ---------------- STATE ---------------- */
    boolean intakeOn = false;
    boolean intakeToggleReady = true;

    boolean targetIsBackCell = true;
    long lastVisionUpdateTime = 0;
    long lastManualOverrideTime = 0;
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
        // The flywheel shoots with NEGATIVE power (autos, Turrettest); reverse it so the RPM math here stays positive.
        flywheel.setDirection(DcMotor.Direction.REVERSE);
        feedingMotor = hardwareMap.dcMotor.get(FEEDING_MOTOR_NAME);
        feedingMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        feedingMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        feedingMotor.setPower(0);
        flywheelPID = new PIDController(0.001, 0, 0); // TODO: your tuned gains

        follower = Constants.create(hardwareMap); // Pedro 3
        follower.setPose(new Pose(72, 12, Math.toRadians(90))); // TODO: match real match start

        aprilTag = new AprilTagProcessor.Builder().build();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, WEBCAM_NAME))
                .addProcessor(aprilTag)
                .build();

        telemetry.addLine("Initialized. GP2: B intake, LB auto/manual, RT fire.");
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

        TurretAim.Result aim = TurretAim.aim(pose.x(), pose.y(), pose.heading(), TurretAim.RED, targetIsBackCell);
        double turretAngle = aim.turretDeg;
        double servoPos = applyVisionTrim(aim.servoPos, pose);

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

        double distance = getShooterDistance(pose, aim.distanceIn);
        updateShooterForDistance(distance);

        telemetry.addData("Hive target", targetIsBackCell ? "BACK" : "AUDIENCE");
        telemetry.addData("Turret", "%.1f deg, servo %.3f%s", aim.turretDeg, servoPos, aim.outOfRange ? " (OUT OF RANGE)" : "");
        telemetry.addData("Distance", distance);
    }

    // Tags face down only when that cell's opening faces up - seeing a cell's
    // tags means aim at that same cell, not the other one.
    private void updateHiveCellTarget(Pose pose) {
        boolean sawBack = false, sawAudience = false;
        for (AprilTagDetection d : aprilTag.getDetections()) {
            for (int id : BACK_CELL_TAGS)     if (tagId(d) == id) sawBack = true;
            for (int id : AUDIENCE_CELL_TAGS) if (tagId(d) == id) sawAudience = true;
        }
        if (sawBack && !sawAudience) { targetIsBackCell = true;  lastVisionUpdateTime = System.currentTimeMillis(); }
        else if (sawAudience && !sawBack) { targetIsBackCell = false; lastVisionUpdateTime = System.currentTimeMillis(); }

        if (gamepad2.dpad_up)   { targetIsBackCell = true;  lastManualOverrideTime = System.currentTimeMillis(); }
        if (gamepad2.dpad_down) { targetIsBackCell = false; lastManualOverrideTime = System.currentTimeMillis(); }

        // No recent tag or manual input: same rule as TurretAim and the autos (lower half = audience CELL).
        boolean visionFresh = System.currentTimeMillis() - lastVisionUpdateTime < VISION_STALE_MS;
        boolean recentOverride = System.currentTimeMillis() - lastManualOverrideTime < MANUAL_OVERRIDE_GRACE_MS;
        if (!visionFresh && !recentOverride) {
            targetIsBackCell = pose.y() >= FIELD_HALF_DIVIDER;
        }
    }

    // Small live correction on top of the odometry aim, only once past the
    // half-field line and a hive tag is actually visible - odometry drifts
    // (especially after a collision), a visible tag doesn't.
    private double applyVisionTrim(double odometryServoPos, Pose pose) {
        if (pose.y() <= FIELD_HALF_DIVIDER) return odometryServoPos;
        for (AprilTagDetection d : aprilTag.getDetections()) {
            if (!containsId(BACK_CELL_TAGS, tagId(d)) && !containsId(AUDIENCE_CELL_TAGS, tagId(d))) continue;
            double trim = Range.clip(TurretAim.TURRET_SIGN * d.ftcPose.bearing * VISION_TRIM_GAIN, -MAX_VISION_TRIM_PER_LOOP, MAX_VISION_TRIM_PER_LOOP);
            return Range.clip(odometryServoPos + trim, TURRET_MIN_POS, TURRET_MAX_POS);
        }
        return odometryServoPos;
    }

    private double getShooterDistance(Pose pose, double odometryDistance) {
        if (pose.y() <= FIELD_HALF_DIVIDER) return odometryDistance;
        for (AprilTagDetection d : aprilTag.getDetections()) {
            if (containsId(BACK_CELL_TAGS, tagId(d)) || containsId(AUDIENCE_CELL_TAGS, tagId(d))) return d.ftcPose.range;
        }
        return odometryDistance;
    }

    // FTC SDK 12.0: AprilTagDetection is now a base class; only single tags have an id.
    // Cluster detections return -1 here (so they are ignored by the tag-id matching above).
    // TODO(pedro3): if the BIOBUZZ library returns clusters, match on clusterDet.metadata instead.
    private int tagId(AprilTagDetection d) {
        return (d instanceof AprilTagSingleDetection) ? ((AprilTagSingleDetection) d).id : -1;
    }

    private boolean containsId(int[] ids, int id) {
        for (int x : ids) if (x == id) return true;
        return false;
    }

    private boolean turretReady() {
        return System.currentTimeMillis() - lastAngleChangeTime > TURRET_SETTLE_MS && flywheelAtSpeed;
    }

    // Blends between the nearest measured distance/RPM points in the table.
    private void updateShooterForDistance(double distance) {
        double targetRPM = interpolate(shotTable, distance, 1);

        double currentRPM = flywheel.getVelocity() / FLYWHEEL_TICKS_PER_REV * 60.0;
        double power = flywheelPID.calculate(targetRPM, targetRPM - currentRPM); // Pedro 3 PIDController takes (target, error)
        flywheel.setPower(power);
        flywheelAtSpeed = Math.abs(targetRPM - currentRPM) < FLYWHEEL_RPM_TOLERANCE;

        telemetry.addData("Target RPM", targetRPM);
    }

    private double interpolate(double[][] table, double distance, int column) {
        if (distance <= table[0][0]) return table[0][column];
        if (distance >= table[table.length - 1][0]) return table[table.length - 1][column];
        for (int i = 0; i < table.length - 1; i++) {
            double d0 = table[i][0], d1 = table[i + 1][0];
            if (distance >= d0 && distance <= d1) {
                double t = (distance - d0) / (d1 - d0);
                return table[i][column] + t * (table[i + 1][column] - table[i][column]);
            }
        }
        return table[table.length - 1][column];
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
        if (visionPortal != null) visionPortal.close();
    }
}
