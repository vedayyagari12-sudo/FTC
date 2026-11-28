package org.firstinspires.ftc.teamcode;

import com.pedropathing.math.Pose;

/**
 * RobotConfig - every number and name the OpModes share, in one place. Change it here and every auto and
 * TeleOp picks it up. OpModes use these through: import static org.firstinspires.ftc.teamcode.RobotConfig.*;
 *
 * Two other files hold the rest of the robot's settings:
 *   TurretAim.java        - turret servo center / range / 270 degree limit / direction, turret pivot offset,
 *                           HIVE and CELL positions on the field.
 *   pedro/Constants.java  - drivetrain motor names and directions, Pinpoint pod offsets, tuned follower values
 *                           (AutoTune output is pasted there).
 *
 * Field frame (Pedro Pathing 3): inches, origin at the bottom-left corner, audience wall y = 0, red wall x = 0,
 * heading in degrees counter-clockwise from +x (90 = facing away from the audience).
 */
public class RobotConfig {

    /* ===================== STARTING POSITIONS (inches, degrees) ===================== */
    // Where the robot is placed before INIT. The full auto and the park auto for the same spot share these.
    public static final double CLOSE_RED_START_X  = 61,  CLOSE_RED_START_Y  = 8,   CLOSE_RED_START_HEADING_DEG  = 90;
    public static final double CLOSE_BLUE_START_X = 83,  CLOSE_BLUE_START_Y = 136, CLOSE_BLUE_START_HEADING_DEG = 270;
    public static final double FAR_RED_START_X    = 64,  FAR_RED_START_Y    = 142, FAR_RED_START_HEADING_DEG    = 270;
    public static final double FAR_BLUE_START_X   = 80,  FAR_BLUE_START_Y   = 2,   FAR_BLUE_START_HEADING_DEG   = 90;

    // TeleOp start. Used when no auto ran before it (practice), or when USE_AUTO_END_POSE is false.
    public static final double TELEOP_START_X = 72, TELEOP_START_Y = 12, TELEOP_START_HEADING_DEG = 90; // TODO: match where you place the robot
    // true: TeleOp starts from wherever the auto ended (saved in the auto's stop()), so the turret aim stays right.
    public static final boolean USE_AUTO_END_POSE = true;

    /** Where the last auto ended, or null. Set by every auto's stop(); read and cleared by the TeleOp. */
    public static Pose autoEndPose = null;

    public static void saveAutoEndPose(Pose p) {
        autoEndPose = new Pose(p.x(), p.y(), p.heading()); // copy, not the follower's live object
    }

    /* ===================== HARDWARE NAMES (26season config, see HARDWARE_CONFIG.md) ===================== */
    // Drive motors and "pinpoint" are also named in pedro/Constants.java; keep them the same there.
    public static final String FRONT_LEFT_NAME    = "FrontLeft";
    public static final String FRONT_RIGHT_NAME   = "FrontRight";
    public static final String BACK_LEFT_NAME     = "BackLeft";
    public static final String BACK_RIGHT_NAME    = "BackRight";
    public static final String FLYWHEEL_NAME      = "flywheel";      // Expansion Hub motor 0
    public static final String INTAKE_MOTOR_NAME  = "intakemotor";   // Expansion Hub motor 1
    public static final String FEEDING_MOTOR_NAME = "feedingmotor";  // Expansion Hub motor 2
    public static final String RAMP_SERVO_NAME    = "rampServo";     // Expansion Hub servo port 0, standard Servo, lowered by RampControl
    public static final String TURRET_SERVO_NAME  = "turretServo";   // Expansion Hub servo 1, standard Servo
    public static final String INTAKE_SERVO_NAME  = "Intakeservo";   // Control Hub servo 0, continuous rotation (capital I)

    /* ===================== FLOWER RAMP (rampServo) ===================== */
    // Two servos on one signal wire. It rests UP (default) at power-on. TeleOp: HOLD gamepad 2 left trigger to put it
    // on the floor, let go and it goes back up. Autos: down at the flower intake pose, back up after the intake.
    // TODO: measure both numbers with the "Ramp Test" TeleOp. While they are equal the code never moves the ramp.
    public static final double RAMP_DOWN_POS = 0.62;  // floor. Positions use the WIDE 500-2500us pulse range (see RampControl), so this is about the old 0.63
    public static final double RAMP_UP_POS   = 0.0;   // up position. Find it with Ramp Test: the highest value where the ramp still moves freely (not jammed)
    public static final double RAMP_SPEED    = 1.0;   // how fast the ramp moves, in servo position units per second (bigger = faster)
    public static final double RAMP_TRIGGER  = 0.5;   // left trigger must be pushed past this to lower the ramp

    /* ===================== POWERS (-1 to 1; flip the sign if a part runs backwards) ===================== */
    public static final double INTAKE_MOTOR_POWER = 1.0;   // same as Turrettest, which was flipped to this on the robot
    public static final double INTAKE_SERVO_POWER = -1.0;  // from IntakeTeleOp's servo-fix PR; flip if it pushes balls out
    public static final double FEED_POWER         = -1.0;  // feeding motor while shooting

    /* ===================== FLYWHEEL ===================== */
    public static final double FLYWHEEL_DIRECTION     = -1.0;  // the flywheel shoots with NEGATIVE power on this robot
    public static final double FLYWHEEL_TICKS_PER_REV = 28.0;  // TODO confirm: encoder ticks per flywheel turn (bare goBILDA motor = 28)
    public static final double FLYWHEEL_RPM_TOLERANCE = 100;   // "at speed" = within this many RPM of the target
    // Autos wait for the flywheel before Path 1: at least REV_MIN_MS, at most REV_TIMEOUT_MS.
    public static final double REV_MIN_MS     = 800;
    public static final double REV_TIMEOUT_MS = 3000;
    // FlywheelPIDF gains (Red/Blue TeleOp and Turret Test). F = power per target RPM, about 1 / max RPM.
    // Tuned 2026-11-25 with the Flywheel Tuner Data TeleOp + flywheel-pidf-tuner.html:
    // measured K = 5261 RPM at full power, tau = 0.81 s, noise +/-32 RPM, loop 6 ms, ball dip 70 RPM at 2599 RPM.
    public static final double FLYWHEEL_P = 0.00135, FLYWHEEL_I = 0.0000833, FLYWHEEL_D = 0;
    public static final double FLYWHEEL_F = 0.00019;
    public static final double FLYWHEEL_MAX_RPM = 6000; // TODO confirm: free speed of the flywheel motor (6000 for a bare goBILDA 5203)
    // TeleOp shot table: {flat distance from the flywheel shaft to the CELL opening in inches, flywheel RPM,
    // ramp/hood position (unused for now)}. Distances in between are blended; past the first or last row the
    // TeleOp holds that row's RPM. Fill it from shot-table.html (~/Desktop/BioBuzz-Turret).
    // TeleOps and autos both pick their RPM here from the odometry distance (shotRpm below). With only one row the
    // answer is that row's RPM at every distance; each tested row added makes the RPM follow distance.
    public static final double[][] SHOT_TABLE = {
        {32, 2700, 0},  // tested 2026-11-25: flywheel shaft to CELL opening 32 in, 45% in Turret Test, shots consistent
    };

    /** Flywheel RPM for a flat distance (inches, flywheel shaft to CELL), blended between SHOT_TABLE rows. */
    public static double shotRpm(double distanceIn) {
        double[][] t = SHOT_TABLE;
        if (distanceIn <= t[0][0]) return t[0][1];
        for (int i = 1; i < t.length; i++) {
            if (distanceIn <= t[i][0]) {
                double f = (distanceIn - t[i - 1][0]) / (t[i][0] - t[i - 1][0]);
                return t[i - 1][1] + f * (t[i][1] - t[i - 1][1]);
            }
        }
        return t[t.length - 1][1]; // past the last row: hold its RPM
    }

    /* ===================== DRIVER CONTROLS ===================== */
    public static final double TURN_SCALE     = 0.6;  // right stick turn speed (1.0 = full)
    public static final double STICK_DEADZONE = 0.1;  // stick movement smaller than this is ignored
}
