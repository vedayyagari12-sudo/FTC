package org.firstinspires.ftc.teamcode;

import static com.pedropathing.api.Paths.line;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedro.Constants;

/**
 * Far Blue Dead Auto  -  drives to the parking zone and stops. No flywheel, no intake.
 * Same end point as the full auto for this side. No flywheel, no intake.
 * Place the robot at the start pose (80, 2) facing 90 degrees before INIT.
 * Edit the points below if the robot parks in the wrong spot.
 */
@Autonomous(name = "Far Blue Dead Auto", group = "Auto")
public class FarBlueAlliancePark extends OpMode {

    /* ---------------- START POSE ---------------- */
    static final double START_X = 80;
    static final double START_Y = 2;
    static final double START_HEADING_DEG = 90;

    /* ---------------- DELAY BEFORE MOVING (milliseconds) ---------------- */
    static final double START_DELAY_MS = 0;

    /* ---------------- PATH POINTS ---------------- */
    static final double[] P1 = {139.8, 18.9, 90, 90}; // x, y, start heading, end heading

    private static final double[][] POINTS = {P1};

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

        paths = new Path[POINTS.length];
        Pose from = start;
        for (int i = 0; i < POINTS.length; i++) {
            double[] p = POINTS[i];
            Pose to = new Pose(p[0], p[1], Math.toRadians(p[3]));
            paths[i] = line(from, to).linear(Math.toRadians(p[2]), Math.toRadians(p[3]));
            from = to;
        }

        telemetry.addLine("Far Blue Dead Auto ready.");
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
