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
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose; // Pedro 2.x: com.pedropathing.geometry.Pose
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

@TeleOp(name = "Red TeleOp Full")
public class RedTeleOp extends OpMode {

    /* ---------------- HARDWARE MAP NAMES (update to match your config) ---------------- */
    final String FRONT_LEFT_NAME    = "FrontLeft";
    final String FRONT_RIGHT_NAME   = "FrontRight";
    final String BACK_LEFT_NAME     = "BackLeft";
    final String BACK_RIGHT_NAME    = "BackRight";
    final String INTAKE_MOTOR_NAME  = "intakemotor";
    final String INTAKE_SERVO1_NAME = "intakeservo";
    final String INTAKE_SERVO2_NAME = "intakeservo2";
    final String TURRET_SERVO1_NAME = "turretServo1";
    final String TURRET_SERVO2_NAME = "turretServo2";
    final String HOOD_SERVO_NAME    = "hoodServo";
    final String FLYWHEEL_NAME      = "flywheel";
    final String WEBCAM_NAME        = "Webcam 1";

    /* ---------------- HARDWARE ---------------- */
    DcMotor FrontLeftWheel, FrontRightWheel, BackLeftWheel, BackRightWheel;
    DcMotor IntakeMotor;
    CRServo IntakeServo1, IntakeServo2;

    Servo turretServo1, turretServo2;
    Servo hood;
    DcMotorEx flywheel;
    Follower follower;
    AprilTagProcessor aprilTag;
    VisionPortal visionPortal;
    PIDController flywheelPID; // TODO: swap in your actual PID class if the name/signature differs

    /* ---------------- DRIVE / INTAKE TUNING ---------------- */
    final double INTAKE_MOTOR_POWER  = -1.0;
    final double INTAKE_SERVO1_POWER =  1.0;
    final double INTAKE_SERVO2_POWER = -1.0;
    final double TURN_SCALE = 0.6;

    /* ---------------- HIVE / TAG CONFIG (red only) ---------------- */
    // Each cell's AprilTag faces down toward the tiles only when that cell's
    // opening faces up - so seeing a cell's tags means aim at THAT cell.
    final int[] BACK_CELL_TAGS     = {0, 1, 2, 3};
    final int[] AUDIENCE_CELL_TAGS = {4, 5, 6, 7};

    // Field coordinates - PLACEHOLDERS, measure from your real field.
    final double BACK_CELL_X = 100, BACK_CELL_Y = 48;
    final double AUDIENCE_CELL_X = 100, AUDIENCE_CELL_Y = 73.5;

    final long VISION_STALE_MS = 1000;
    final long MANUAL_OVERRIDE_GRACE_MS = 1000;

    // Field-half fallback guess used only while no tag has been seen recently.
    final double FIELD_HALF_DIVIDER = 70.75;
    final boolean LOWER_HALF_IS_BACK_CELL = true; // verify by testing, may need to flip

    /* ---------------- TURRET CONFIG ---------------- */
    final double TURRET_PAN_MIN_DEG = -90, TURRET_PAN_MAX_DEG = 90;
    final double TURRET_PAN_MIN_POS = 0.0, TURRET_PAN_MAX_POS = 1.0;
    final double TURRET_TOLERANCE_DEG = 2.0;
    final long TURRET_SETTLE_MS = 150;

    // Fine vision trim - only applied past FIELD_HALF_DIVIDER, on top of the
    // odometry aim, when a hive tag is actually visible.
    final double VISION_TRIM_GAIN = 0.02;
    final double MAX_VISION_TRIM_PER_LOOP = 0.01;

    /* ---------------- MANUAL MODE CONFIG ---------------- */
    final double MANUAL_TURRET_RATE = 0.01;
    final double MANUAL_HOOD_RATE = 0.01;
    final double HOOD_MIN_POS = 0.0, HOOD_MAX_POS = 1.0;
    final double STICK_DEADZONE = 0.1;

    /* ---------------- SHOOTER CONFIG ---------------- */
    // {distance in, RPM, hood pos} - PLACEHOLDER rows, replace with tested values.
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
    double manualTurretPos = 0.5;
    double manualHoodPos = 0.3;
    double lastTurretPos = 0.5;
    double lastHoodPos = 0.3;

    /* ---------------- INIT ---------------- */
    @Override
    public void init() {
        FrontLeftWheel  = hardwareMap.dcMotor.get(FRONT_LEFT_NAME);
        FrontRightWheel = hardwareMap.dcMotor.get(FRONT_RIGHT_NAME);
        BackLeftWheel   = hardwareMap.dcMotor.get(BACK_LEFT_NAME);
        BackRightWheel  = hardwareMap.dcMotor.get(BACK_RIGHT_NAME);

        IntakeMotor  = hardwareMap.dcMotor.get(INTAKE_MOTOR_NAME);
        IntakeServo1 = hardwareMap.get(CRServo.class, INTAKE_SERVO1_NAME);
        IntakeServo2 = hardwareMap.get(CRServo.class, INTAKE_SERVO2_NAME);

        BackLeftWheel.setDirection(DcMotor.Direction.REVERSE);
        FrontLeftWheel.setDirection(DcMotor.Direction.REVERSE);

        FrontLeftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontRightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackLeftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        IntakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        IntakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        stopIntake();

        turretServo1 = hardwareMap.get(Servo.class, TURRET_SERVO1_NAME);
        turretServo2 = hardwareMap.get(Servo.class, TURRET_SERVO2_NAME);
        hood         = hardwareMap.get(Servo.class, HOOD_SERVO_NAME);
        flywheel     = hardwareMap.get(DcMotorEx.class, FLYWHEEL_NAME);
        // If one turret servo fights the other, flip this to REVERSE.
        turretServo2.setDirection(Servo.Direction.FORWARD);
        flywheelPID = new PIDController(0.001, 0, 0); // TODO: your tuned gains

        follower = new Follower(hardwareMap); // Pedro 2.x: follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 12, Math.toRadians(90))); // TODO: match real match start

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
        double den = Math.max(1.0, Math.abs(y) + Math.abs(x) + Math.abs(rx));

        FrontLeftWheel.setPower((y + x + rx) / den);
        FrontRightWheel.setPower((y - x - rx) / den);
        BackLeftWheel.setPower((y - x + rx) / den);
        BackRightWheel.setPower((y + x - rx) / den);

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
            // Start manual from wherever auto left the turret/hood so nothing jumps.
            manualTurretPos = lastTurretPos;
            manualHoodPos = lastHoodPos;
        }

        /* -------- TURRET / SHOOTER -------- */
        updateTurretAndShooter();

        // Fire on gamepad2 RT in both modes. Auto waits for aim + flywheel,
        // manual only waits for flywheel since the driver is aiming.
        boolean readyToFire = manualMode ? flywheelAtSpeed : turretReady();
        if (gamepad2.right_trigger > 0.5 && readyToFire) {
            // TODO: trigger your feed/spindexer mechanism here
        }

        telemetry.addData("Aim mode", manualMode ? "MANUAL" : "AUTO");
        telemetry.addData("Ready to fire", readyToFire);

        telemetry.addData("Intake", intakeOn ? "RUNNING" : "STOPPED");
        telemetry.update();
    }

    /* ---------------- TURRET / SHOOTER LOGIC ---------------- */
    private void updateTurretAndShooter() {
        follower.update();
        Pose pose = follower.getPose();

        updateHiveCellTarget(pose);

        double targetX = targetIsBackCell ? BACK_CELL_X : AUDIENCE_CELL_X;
        double targetY = targetIsBackCell ? BACK_CELL_Y : AUDIENCE_CELL_Y;
        double odometryDistance = Math.hypot(targetX - pose.getX(), targetY - pose.getY());

        double fieldAngle = Math.toDegrees(Math.atan2(targetY - pose.getY(), targetX - pose.getX()));
        double turretAngle = Range.clip(normalizeAngle(fieldAngle - Math.toDegrees(pose.getHeading())),
                TURRET_PAN_MIN_DEG, TURRET_PAN_MAX_DEG);
        double servoPos = (turretAngle - TURRET_PAN_MIN_DEG) / (TURRET_PAN_MAX_DEG - TURRET_PAN_MIN_DEG)
                * (TURRET_PAN_MAX_POS - TURRET_PAN_MIN_POS) + TURRET_PAN_MIN_POS;

        servoPos = applyVisionTrim(servoPos, pose);

        if (manualMode) {
            double turn = Math.abs(gamepad2.right_stick_x) > STICK_DEADZONE ? gamepad2.right_stick_x : 0;
            manualTurretPos = Range.clip(manualTurretPos + turn * MANUAL_TURRET_RATE, TURRET_PAN_MIN_POS, TURRET_PAN_MAX_POS);
            servoPos = manualTurretPos;
        }
        turretServo1.setPosition(servoPos);
        turretServo2.setPosition(servoPos);
        lastTurretPos = servoPos;

        if (Double.isNaN(lastCommandedAngle) || Math.abs(turretAngle - lastCommandedAngle) > TURRET_TOLERANCE_DEG) {
            lastAngleChangeTime = System.currentTimeMillis();
        }
        lastCommandedAngle = turretAngle;

        double distance = getShooterDistance(pose, odometryDistance);
        updateShooterForDistance(distance);

        telemetry.addData("Hive target", targetIsBackCell ? "BACK" : "AUDIENCE");
        telemetry.addData("Distance", distance);
    }

    // Tags face down only when that cell's opening faces up - seeing a cell's
    // tags means aim at that same cell, not the other one.
    private void updateHiveCellTarget(Pose pose) {
        boolean sawBack = false, sawAudience = false;
        for (AprilTagDetection d : aprilTag.getDetections()) {
            for (int id : BACK_CELL_TAGS)     if (d.id == id) sawBack = true;
            for (int id : AUDIENCE_CELL_TAGS) if (d.id == id) sawAudience = true;
        }
        if (sawBack && !sawAudience) { targetIsBackCell = true;  lastVisionUpdateTime = System.currentTimeMillis(); }
        else if (sawAudience && !sawBack) { targetIsBackCell = false; lastVisionUpdateTime = System.currentTimeMillis(); }

        if (gamepad2.dpad_up)   { targetIsBackCell = true;  lastManualOverrideTime = System.currentTimeMillis(); }
        if (gamepad2.dpad_down) { targetIsBackCell = false; lastManualOverrideTime = System.currentTimeMillis(); }

        // No recent tag or manual input: guess from which half of the field we're on.
        boolean visionFresh = System.currentTimeMillis() - lastVisionUpdateTime < VISION_STALE_MS;
        boolean recentOverride = System.currentTimeMillis() - lastManualOverrideTime < MANUAL_OVERRIDE_GRACE_MS;
        if (!visionFresh && !recentOverride) {
            boolean onLowerHalf = pose.getY() < FIELD_HALF_DIVIDER;
            targetIsBackCell = (onLowerHalf == LOWER_HALF_IS_BACK_CELL);
        }
    }

    // Small live correction on top of the odometry aim, only once past the
    // half-field line and a hive tag is actually visible - odometry drifts
    // (especially after a collision), a visible tag doesn't.
    private double applyVisionTrim(double odometryServoPos, Pose pose) {
        if (pose.getY() <= FIELD_HALF_DIVIDER) return odometryServoPos;
        for (AprilTagDetection d : aprilTag.getDetections()) {
            if (!containsId(BACK_CELL_TAGS, d.id) && !containsId(AUDIENCE_CELL_TAGS, d.id)) continue;
            double trim = Range.clip(d.ftcPose.bearing * VISION_TRIM_GAIN, -MAX_VISION_TRIM_PER_LOOP, MAX_VISION_TRIM_PER_LOOP);
            return Range.clip(odometryServoPos + trim, TURRET_PAN_MIN_POS, TURRET_PAN_MAX_POS);
        }
        return odometryServoPos;
    }

    private double getShooterDistance(Pose pose, double odometryDistance) {
        if (pose.getY() <= FIELD_HALF_DIVIDER) return odometryDistance;
        for (AprilTagDetection d : aprilTag.getDetections()) {
            if (containsId(BACK_CELL_TAGS, d.id) || containsId(AUDIENCE_CELL_TAGS, d.id)) return d.ftcPose.range;
        }
        return odometryDistance;
    }

    private boolean containsId(int[] ids, int id) {
        for (int x : ids) if (x == id) return true;
        return false;
    }

    private boolean turretReady() {
        return System.currentTimeMillis() - lastAngleChangeTime > TURRET_SETTLE_MS && flywheelAtSpeed;
    }

    private double normalizeAngle(double deg) {
        while (deg > 180) deg -= 360;
        while (deg < -180) deg += 360;
        return deg;
    }

    // Blends between the nearest measured distance/RPM/hood points in the table.
    private void updateShooterForDistance(double distance) {
        double targetRPM = interpolate(shotTable, distance, 1);
        double hoodPos = interpolate(shotTable, distance, 2);

        if (manualMode) {
            double lift = Math.abs(gamepad2.left_stick_y) > STICK_DEADZONE ? -gamepad2.left_stick_y : 0;
            manualHoodPos = Range.clip(manualHoodPos + lift * MANUAL_HOOD_RATE, HOOD_MIN_POS, HOOD_MAX_POS);
            hoodPos = manualHoodPos;
        }
        hood.setPosition(hoodPos);
        lastHoodPos = hoodPos;

        double currentRPM = flywheel.getVelocity() / FLYWHEEL_TICKS_PER_REV * 60.0;
        double power = flywheelPID.calculate(targetRPM, currentRPM); // TODO: match your PID class's real method
        flywheel.setPower(power);
        flywheelAtSpeed = Math.abs(targetRPM - currentRPM) < FLYWHEEL_RPM_TOLERANCE;

        telemetry.addData("Target RPM", targetRPM);
        telemetry.addData("Hood", hoodPos);
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
        IntakeServo1.setPower(INTAKE_SERVO1_POWER);
        IntakeServo2.setPower(INTAKE_SERVO2_POWER);
    }

    private void stopIntake() {
        IntakeMotor.setPower(0);
        IntakeServo1.setPower(0);
        IntakeServo2.setPower(0);
    }

    @Override
    public void stop() {
        stopIntake();
        FrontLeftWheel.setPower(0);
        FrontRightWheel.setPower(0);
        BackLeftWheel.setPower(0);
        BackRightWheel.setPower(0);
        if (visionPortal != null) visionPortal.close();
    }
}
