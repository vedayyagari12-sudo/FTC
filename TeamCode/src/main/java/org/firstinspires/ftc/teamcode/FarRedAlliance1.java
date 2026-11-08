package org.firstinspires.ftc.teamcode;

import static com.pedropathing.api.Paths.line;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedro.Constants;

/**
 * Far Red Alliance 1  -  the field is rotated 180 degrees from blue: x -> 144 - x, y -> 144 - y, heading + 180.
 *
 * Path (field inches):
 *   Start   (64, 142)
 *   Path 1  -> (57, 113)      flywheel shoots 2 s
 *   Path 2  -> (49.7, 133.8)  intake 3 s
 *   Path 3  -> (57, 113)      flywheel shoots 2 s
 *   Path 4  -> (-3.8, 125.1) done
 *
 * Order: wait 5 s (partner shoots first) -> flywheel revs up (robot waits) -> Path 1 -> shoot -> Path 2 -> intake -> Path 3 -> shoot -> Path 4.
 *
 * !! HEADINGS: the screenshot did not show the start/end headings, so every path keeps the SAME heading
 * (HEADING_DEG) and the robot never spins. Change HEADING_DEG (or the per-path values) to match your .pp file.
 * !! What happens at each stop (shoot / intake) is my guess - change the times below.
 */
@Autonomous(name = "Far Red Alliance 1", group = "Auto")
public class FarRedAlliance1 extends OpMode {

    /* ---------------- START POSE ---------------- */
    static final double START_X = 64;
    static final double START_Y = 142;

    /* ---------------- HEADINGS (degrees) - EDIT TO MATCH YOUR .pp ---------------- */
    static final double START_HEADING_DEG = 270;
    static final double P1_HEADING_DEG = 270;
    static final double P2_HEADING_DEG = 90;   // turns around (180) so the robot's BACK faces the flower
    static final double P3_HEADING_DEG = 270;   // turns back around
    static final double P4_HEADING_DEG = 270;

    /* ---------------- PATH POINTS ---------------- */
    static final double P1_X = 57,    P1_Y = 113;
    static final double P2_X = 49.7,  P2_Y = 133.8;
    static final double P3_X = 57,    P3_Y = 113;
    static final double P4_X = -3.8,  P4_Y = 125.1;

    /* ---------------- WAIT AFTER EACH PATH (milliseconds) ---------------- */
    // Path 1 = shoot, Path 2 = intake, Path 3 = shoot, Path 4 = nothing.
    static final double[] WAIT_AFTER_MS = {2000, 3000, 2000, 200};

    /* ---------------- START DELAY: let the alliance partner shoot first ---------------- */
    /* ---------------- AT THE FLOWER ---------------- */
    static final double FLOWER_FORWARD_IN = 1;  // drive this far forward while intaking

    static final double START_DELAY_MS = 5000;

    /* ---------------- FLYWHEEL REV-UP WAIT (before Path 1 starts) ---------------- */
    static final double REV_MIN_MS = 800;
    static final double REV_TIMEOUT_MS = 3000;
    static final double REV_TOLERANCE_RPM = 100;

    /* ---------------- MECHANISMS (same names/values as the close autos) ---------------- */
    static final String TURRET_SERVO_NAME = "turretServo"; // same name as Turrettest - TODO confirm in the robot config
    static final int ALLIANCE = TurretAim.RED;           // which HIVE the turret aims at
    static final String FLYWHEEL_NAME = "flywheel";
    static final String INTAKE_NAME   = "intakemotor";
    static final double FLYWHEEL_RPM  = 2200;
    static final double FLYWHEEL_TICKS_PER_REV = 28.0; // from RedTeleOp - TODO confirm for your motor
    static final double FLYWHEEL_DIRECTION = -1.0;     // flywheel spins with NEGATIVE power
    static final String FEEDING_NAME  = "feedingmotor";
    static final double FEED_POWER    = -1.0;           // feeding motor power while shooting
    static final double INTAKE_POWER = 1.0;

    private enum State { START_DELAY, REV_WAIT, DRIVING, FLOWER_FORWARD, WAITING, DONE }

    private Follower follower;
    private DcMotorEx flywheel;
    private DcMotor intakeMotor;
    private DcMotor feedingMotor;
    private Servo turretServo;
    private TurretAim.Result aim; // latest turret aim, shown in telemetry
    private Path[] paths;
    private int step = 0;
    private State state = State.DONE;
    private final ElapsedTime waitTimer = new ElapsedTime();

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);

        Pose start = new Pose(START_X, START_Y, Math.toRadians(START_HEADING_DEG));
        Pose p1 = new Pose(P1_X, P1_Y, Math.toRadians(P1_HEADING_DEG));
        Pose p2 = new Pose(P2_X, P2_Y, Math.toRadians(P2_HEADING_DEG));
        Pose p3 = new Pose(P3_X, P3_Y, Math.toRadians(P3_HEADING_DEG));
        Pose p4 = new Pose(P4_X, P4_Y, Math.toRadians(P4_HEADING_DEG));

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

        turretServo = hardwareMap.get(Servo.class, TURRET_SERVO_NAME);
        turretServo.setPosition(TurretAim.TURRET_CENTER);

        paths = new Path[] {
                line(start, p1).linear(Math.toRadians(START_HEADING_DEG), Math.toRadians(P1_HEADING_DEG)),
                line(p1, p2).linear(Math.toRadians(P1_HEADING_DEG), Math.toRadians(P2_HEADING_DEG)),
                line(p2, p3).linear(Math.toRadians(P2_HEADING_DEG), Math.toRadians(P3_HEADING_DEG)),
                line(p3, p4).linear(Math.toRadians(P3_HEADING_DEG), Math.toRadians(P4_HEADING_DEG))
        };

        telemetry.addLine("Far Red Alliance 1 ready.");
        telemetry.addData("Start pose", "(%.0f, %.0f) heading %.0f deg", START_X, START_Y, START_HEADING_DEG);
        telemetry.addLine("Place the robot at the start pose, then press Start.");
        telemetry.update();
    }

    @Override
    public void start() {
        follower.setPose(new Pose(START_X, START_Y, Math.toRadians(START_HEADING_DEG)));
        follower.update();
        aimTurret();
        step = 0;
        // Wait first so the alliance partner can shoot, then rev the flywheel and drive.
        waitTimer.reset();
        state = State.START_DELAY;
    }

    @Override
    public void loop() {
        follower.update();
        aimTurret();

        switch (state) {
            case START_DELAY:
                if (waitTimer.milliseconds() >= START_DELAY_MS) {
                    flywheel.setVelocity(FLYWHEEL_DIRECTION * FLYWHEEL_RPM / 60.0 * FLYWHEEL_TICKS_PER_REV);
                    waitTimer.reset();
                    state = State.REV_WAIT;
                }
                break;

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
                    runActionAfterPath(step);
                    waitTimer.reset();
                    if (step == 1) {
                        // At the flower (back already turned to it): intake is on, creep forward 1 in.
                        followRobotRelative(FLOWER_FORWARD_IN, 0, P2_HEADING_DEG);
                        state = State.FLOWER_FORWARD;
                    } else {
                        state = State.WAITING;
                    }
                }
                break;

            case FLOWER_FORWARD:
                if (follower.atParametricEnd()) {
                    state = State.WAITING; // timer keeps counting from when the intake turned on
                }
                break;

            case WAITING:
                if (waitTimer.milliseconds() >= WAIT_AFTER_MS[step]) {
                    stopActionAfterPath(step);
                    step++;
                    if (step < paths.length) {
                        follower.follow(paths[step]);
                        state = State.DRIVING;
                    } else {
                        state = State.DONE;
                    }
                }
                break;

            case DONE:
                // Pedro keeps holding the last pose.
                break;
        }

        Pose pose = follower.pose();
        telemetry.addData("State", state);
        telemetry.addData("Turret", "%.1f deg, servo %.3f, %.1f in to the %s CELL%s", aim.turretDeg, aim.servoPos, aim.distanceIn,
                aim.targetFar ? "far" : "audience", aim.outOfRange ? " (OUT OF RANGE)" : "");
        telemetry.addData("Path step", "%d of %d", Math.min(step + 1, paths.length), paths.length);
        telemetry.addData("Flywheel RPM", "%.0f (target %.0f)", flywheelRpm(), FLYWHEEL_RPM);
        telemetry.addData("Pose", "x %.1f  y %.1f  heading %.0f deg", pose.x(), pose.y(), Math.toDegrees(pose.heading()));
        telemetry.update();
    }

    /** Short straight move from where the robot is, forward/left in inches from the ROBOT's point of view. */
    private void followRobotRelative(double forwardIn, double leftIn, double headingDeg) {
        Pose now = follower.pose();
        double h = Math.toRadians(headingDeg);
        double dx = forwardIn * Math.cos(h) - leftIn * Math.sin(h);
        double dy = forwardIn * Math.sin(h) + leftIn * Math.cos(h);
        Pose from = new Pose(now.x(), now.y(), h);
        Pose to = new Pose(now.x() + dx, now.y() + dy, h);
        follower.follow(line(from, to).constant(h));
    }

    /** Points the turret at the HIVE CELL for where the robot is right now (see TurretAim.java). */
    private void aimTurret() {
        Pose p = follower.pose();
        aim = TurretAim.aim(p.x(), p.y(), p.heading(), ALLIANCE);
        turretServo.setPosition(aim.servoPos);
    }

    private double flywheelRpm() {
        return Math.abs(flywheel.getVelocity()) / FLYWHEEL_TICKS_PER_REV * 60.0;
    }

    /** Starts the mechanism for this path. It runs during the wait after the path. */
    private void runActionAfterPath(int pathIndex) {
        switch (pathIndex) {
            case 1: // intake
                intakeMotor.setPower(INTAKE_POWER);
                break;
            case 0:
            case 2: // shoot: flywheel is already running, so feed the balls in
                feedingMotor.setPower(FEED_POWER);
                break;
            default: // 3: nothing
                break;
        }
    }

    /** Turns the mechanism off when the wait is over. */
    private void stopActionAfterPath(int pathIndex) {
        switch (pathIndex) {
            case 0: // first shot finished
                feedingMotor.setPower(0);
                break;
            case 1:
                intakeMotor.setPower(0);
                break;
            case 2: // second shot finished
                flywheel.setPower(0);
                feedingMotor.setPower(0);
                break;
        }
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
