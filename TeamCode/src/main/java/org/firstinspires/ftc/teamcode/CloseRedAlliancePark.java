package org.firstinspires.ftc.teamcode;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedro.Constants;

/**
 * Close Red Dead Auto  -  drives to the parking zone and stops. No flywheel, no intake.
 * Same end point as the full auto for this side. No flywheel, no intake.
 * Place the robot at the start pose (61, 136) facing -90 degrees before INIT.
 * Edit the points below if the robot parks in the wrong spot.
 */
@Autonomous(name = "Close Red Dead Auto", group = "Auto")
public class CloseRedAlliancePark extends OpMode {

    /* ---------------- START POSE ---------------- */
    static final double START_X = 61;
    static final double START_Y = 8;
    static final double START_HEADING_DEG = 90;

    /* ---------------- DELAY BEFORE MOVING (milliseconds) ---------------- */
    static final double START_DELAY_MS = 0;

    /* ---------------- PATH POINTS ---------------- */
    // The robot first CURVES wide around the hive support pole, then drives straight FORWARD to the park spot.
    static final double CURVE_CONTROL_X = 10, CURVE_CONTROL_Y = 16; // pulls the curve outward
    static final double WAYPOINT_X = 8, WAYPOINT_Y = 54;   // where the curve ends (clear of the hive)
    static final double PARK_X = 3, PARK_Y = 143.36;           // park spot (same end point as the full auto)
    static final double HEADING_DEG = 90;                                  // heading is held the whole way

    private Follower follower;
    private Path[] paths;
    private int step = 0;
    private boolean started = false;
    private long startTimeMs;

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);

        Pose start = new Pose(START_X, START_Y, Math.toRadians(START_HEADING_DEG));
        follower.setPose(start);

        Pose control = new Pose(CURVE_CONTROL_X, CURVE_CONTROL_Y, Math.toRadians(HEADING_DEG));
        Pose waypoint = new Pose(WAYPOINT_X, WAYPOINT_Y, Math.toRadians(HEADING_DEG));
        Pose park = new Pose(PARK_X, PARK_Y, Math.toRadians(HEADING_DEG));

        paths = new Path[] {
                // 1) curve around the pole
                curve(start, control, waypoint).constant(Math.toRadians(HEADING_DEG)),
                // 2) straight forward into the park zone
                line(waypoint, park).constant(Math.toRadians(HEADING_DEG))
        };

        telemetry.addLine("Close Red Dead Auto ready.");
        telemetry.addData("Start pose", "(%.0f, %.0f) heading %.0f deg", START_X, START_Y, START_HEADING_DEG);
        telemetry.addLine("Place the robot at the start pose, then press Start.");
        telemetry.update();
    }

    @Override
    public void start() {
        follower.setPose(new Pose(START_X, START_Y, Math.toRadians(START_HEADING_DEG)));
        follower.update();
        step = 0;
        started = false;
        startTimeMs = System.currentTimeMillis();
    }

    @Override
    public void loop() {
        follower.update();

        if (!started && System.currentTimeMillis() - startTimeMs >= START_DELAY_MS) {
            started = true;
            follower.follow(paths[0]);
        }

        if (started && step < paths.length && follower.atParametricEnd()) {
            step++;
            if (step < paths.length) follower.follow(paths[step]);
        }

        Pose pose = follower.pose();
        telemetry.addData("Path step", "%d of %d", Math.min(step + 1, paths.length), paths.length);
        telemetry.addData("Pose", "x %.1f  y %.1f  heading %.0f deg", pose.x(), pose.y(), Math.toDegrees(pose.heading()));
        telemetry.update();
    }

    @Override
    public void stop() {
        if (follower != null) {
            follower.stop();
            follower.update();
        }
    }
}
