package org.firstinspires.ftc.teamcode;

import static com.pedropathing.api.Paths.line;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedro.Constants;

/**
 * Close Blue Alliance 1  -  converted from close_blue_allience_1_1.pp (Pedro visualizer v1.5.0).
 *
 * Path (field coordinates in inches, headings in degrees):
 *   Start   (83, 136)  heading 270
 *   Path 1  -> (83, 110)   heading 270 -> 270   flywheel 2200 RPM for 2 s
 *   Path 2  -> (140.2, 68.2)   heading 270 -> 0     intake for 3 s
 *   Path 3  -> (141.0, 0.64) heading 0   -> 180   wait 200 ms after
 *   Then backs up 3 in.
 *
 * Order: flywheel revs up (robot waits) -> Path 1 -> shoot 2 s -> Path 2 -> intake while moving
 * 2 in left then 2 in forward -> Path 3 -> back up 3 in.
 *
 * Left / forward / backward are from the ROBOT's point of view (using the heading at that point).
 *
 * HOW TO RUN: Driver Station -> Autonomous -> "Close Blue Alliance 1" -> INIT -> Start.
 * Put the robot at the START pose below before pressing INIT.
 *
 * The start heading is 270 so the robot does NOT spin at the beginning: it starts already facing the
 * way Path 1 wants. Place the robot facing 270 degrees (toward the bottom of the field in the visualizer)
 * at the start point before pressing INIT.
 */
@Autonomous(name = "Close Blue Alliance 1", group = "Auto")
public class CloseBlueAlliance1 extends OpMode {

    /* ---------------- START POSE ---------------- */
    static final double START_X = 83;
    static final double START_Y = 136;
    static final double START_HEADING_DEG = 270;  // matches Path 1 start heading, so no turn at the beginning

    /* ---------------- PATH POINTS ---------------- */
    // Path 1
    static final double P1_X = 83,  P1_Y = 110;
    static final double P1_START_DEG = 270, P1_END_DEG = 270;
    // Path 2
    static final double P2_X = 140.2, P2_Y = 68.2;
    static final double P2_START_DEG = 270, P2_END_DEG = 0;
    // Path 3
    static final double P3_X = 141.0, P3_Y = 0.64;
    static final double P3_START_DEG = 0,   P3_END_DEG = 180;

    /* ---------------- WAIT AFTER EACH PATH (milliseconds) ---------------- */
    // Path 1 wait = flywheel time, Path 2 wait = intake time, Path 3 = 200.
    static final double FLYWHEEL_MS = 2000;   // flywheel runs 2 s after Path 1
    static final double INTAKE_MS   = 3000;   // intake runs 3 s after Path 2 (at the flower)
    static final double WAIT_AFTER_PATH3_MS = 200;

    /* ---------------- FLYWHEEL REV-UP WAIT (before Path 1 starts) ---------------- */
    static final double REV_MIN_MS = 800;       // always wait at least this long
    static final double REV_TIMEOUT_MS = 3000;  // never wait longer than this
    static final double REV_TOLERANCE_RPM = 100; // "at speed" = within this of 2200

    /* ---------------- SMALL MOVES (inches, robot's point of view) ---------------- */
    static final double FLOWER_LEFT_IN = 2;     // while intaking
    static final double FLOWER_FORWARD_IN = 2;  // while intaking
    static final double BACKUP_IN = 3;          // at the very end

    /* ---------------- MECHANISMS (same names/values as TurretTest + RedTeleOp) ---------------- */
    static final String FLYWHEEL_NAME = "flywheel";
    static final String INTAKE_NAME   = "intakemotor";
    static final double FLYWHEEL_RPM  = 2200;
    static final double FLYWHEEL_TICKS_PER_REV = 28.0; // from RedTeleOp - TODO confirm for your motor
    static final double FLYWHEEL_DIRECTION = -1.0;     // flywheel spins with NEGATIVE power (-0.6 in TurretTest)
    static final String FEEDING_NAME  = "feedingmotor";
    static final double FEED_POWER    = -1.0;           // feeding motor power while shooting
    static final double INTAKE_POWER = 1.0;            // same as TurretTest

    private DcMotorEx flywheel;
    private DcMotor intakeMotor;
    private DcMotor feedingMotor;

    private enum State { REV_WAIT, DRIVING, WAITING, FLOWER_LEFT, FLOWER_FORWARD, INTAKE_WAIT, BACKUP, DONE }

    private Follower follower;
    private Path[] paths;
    private int step = 0;
    private State state = State.DONE;
    private final ElapsedTime intakeTimer = new ElapsedTime();
    private final ElapsedTime waitTimer = new ElapsedTime();

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);

        Pose start = new Pose(START_X, START_Y, Math.toRadians(START_HEADING_DEG));
        Pose p1 = new Pose(P1_X, P1_Y, Math.toRadians(P1_END_DEG));
        Pose p2 = new Pose(P2_X, P2_Y, Math.toRadians(P2_END_DEG));
        Pose p3 = new Pose(P3_X, P3_Y, Math.toRadians(P3_END_DEG));

        flywheel = hardwareMap.get(DcMotorEx.class, FLYWHEEL_NAME);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // uses the attached encoder for speed control
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheel.setPower(0);
        intakeMotor = hardwareMap.dcMotor.get(INTAKE_NAME);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setPower(0);
        feedingMotor = hardwareMap.dcMotor.get(FEEDING_NAME);
        feedingMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        feedingMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        feedingMotor.setPower(0);

        follower.setPose(start);

        paths = new Path[] {
                line(start, p1).linear(Math.toRadians(P1_START_DEG), Math.toRadians(P1_END_DEG)),
                line(p1, p2).linear(Math.toRadians(P2_START_DEG), Math.toRadians(P2_END_DEG)),
                line(p2, p3).linear(Math.toRadians(P3_START_DEG), Math.toRadians(P3_END_DEG))
        };

        telemetry.addLine("Close Blue Alliance 1 ready.");
        telemetry.addData("Start pose", "(%.0f, %.0f) heading %.0f deg", START_X, START_Y, START_HEADING_DEG);
        telemetry.addLine("Place the robot at the start pose, then press Start.");
        telemetry.update();
    }

    @Override
    public void start() {
        follower.setPose(new Pose(START_X, START_Y, Math.toRadians(START_HEADING_DEG)));
        follower.update();
        step = 0;
        // Spin the flywheel up FIRST and wait for it before driving Path 1.
        startFlywheel();
        waitTimer.reset();
        state = State.REV_WAIT;
    }

    @Override
    public void loop() {
        follower.update();

        switch (state) {
            case REV_WAIT: {
                double t = waitTimer.milliseconds();
                boolean atSpeed = Math.abs(flywheelRpm() - FLYWHEEL_RPM) < REV_TOLERANCE_RPM;
                if ((atSpeed && t >= REV_MIN_MS) || t >= REV_TIMEOUT_MS) {
                    follower.follow(paths[0]);
                    state = State.DRIVING;
                }
                break;
            }

            case DRIVING:
                if (follower.atParametricEnd()) {
                    waitTimer.reset();
                    if (step == 0) {
                        // Path 1 done: flywheel is already at speed. Feed the balls in for FLYWHEEL_MS.
                        feedingMotor.setPower(FEED_POWER);
                        state = State.WAITING;
                    } else if (step == 1) {
                        // At the flower: intake on, move left then forward while intaking.
                        intakeMotor.setPower(INTAKE_POWER);
                        intakeTimer.reset();
                        followRobotRelative(0, FLOWER_LEFT_IN, P2_END_DEG); // forward 0, left 2
                        state = State.FLOWER_LEFT;
                    } else {
                        state = State.WAITING;
                    }
                }
                break;

            case WAITING:
                if (step == 0 && waitTimer.milliseconds() >= FLYWHEEL_MS) {
                    flywheel.setPower(0);
                    feedingMotor.setPower(0);
                    step = 1;
                    follower.follow(paths[1]);
                    state = State.DRIVING;
                } else if (step == 2 && waitTimer.milliseconds() >= WAIT_AFTER_PATH3_MS) {
                    followRobotRelative(-BACKUP_IN, 0, P3_END_DEG); // back up 3 in
                    state = State.BACKUP;
                }
                break;

            case FLOWER_LEFT:
                if (follower.atParametricEnd()) {
                    followRobotRelative(FLOWER_FORWARD_IN, 0, P2_END_DEG); // forward 2 in
                    state = State.FLOWER_FORWARD;
                }
                break;

            case FLOWER_FORWARD:
                if (follower.atParametricEnd()) {
                    state = State.INTAKE_WAIT;
                }
                break;

            case INTAKE_WAIT:
                // Keep intaking until INTAKE_MS total has passed since the intake turned on.
                if (intakeTimer.milliseconds() >= INTAKE_MS) {
                    intakeMotor.setPower(0);
                    step = 2;
                    follower.follow(paths[2]);
                    state = State.DRIVING;
                }
                break;

            case BACKUP:
                if (follower.atParametricEnd()) {
                    state = State.DONE;
                }
                break;

            case DONE:
                // Pedro keeps holding the last pose.
                break;
        }

        Pose pose = follower.pose();
        telemetry.addData("State", state);
        telemetry.addData("Flywheel RPM", "%.0f (target %.0f)", flywheelRpm(), FLYWHEEL_RPM);
        telemetry.addData("Pose", "x %.1f  y %.1f  heading %.0f deg", pose.x(), pose.y(), Math.toDegrees(pose.heading()));
        telemetry.update();
    }

    /**
     * Drives a short straight line from where the robot is now, measured from the ROBOT's point of view
     * (forward / left, in inches), keeping the heading fixed at headingDeg.
     */
    private void followRobotRelative(double forwardIn, double leftIn, double headingDeg) {
        Pose now = follower.pose();
        double h = Math.toRadians(headingDeg);
        double dx = forwardIn * Math.cos(h) - leftIn * Math.sin(h);
        double dy = forwardIn * Math.sin(h) + leftIn * Math.cos(h);
        Pose from = new Pose(now.x(), now.y(), h);
        Pose to = new Pose(now.x() + dx, now.y() + dy, h);
        follower.follow(line(from, to).constant(h));
    }

    private double flywheelRpm() {
        return Math.abs(flywheel.getVelocity()) / FLYWHEEL_TICKS_PER_REV * 60.0;
    }

    private void startFlywheel() {
        flywheel.setVelocity(FLYWHEEL_DIRECTION * FLYWHEEL_RPM / 60.0 * FLYWHEEL_TICKS_PER_REV);
    }

    @Override
    public void stop() {
        if (follower != null) {
            follower.stop();
            follower.update();
        }
        if (flywheel != null) flywheel.setPower(0);
        if (intakeMotor != null) intakeMotor.setPower(0);
        if (feedingMotor != null) feedingMotor.setPower(0);
    }
}
