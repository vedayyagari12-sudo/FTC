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
    public static final String RAMP_SERVO_NAME    = "rampServo";     // Expansion Hub servo 0, standard Servo (not driven yet)
    public static final String TURRET_SERVO_NAME  = "turretServo";   // Expansion Hub servo 1, standard Servo
    public static final String INTAKE_SERVO_NAME  = "Intakeservo";   // Control Hub servo 0, continuous rotation (capital I)

    /* ===================== POWERS (-1 to 1; flip the sign if a part runs backwards) ===================== */
    public static final double INTAKE_MOTOR_POWER = 1.0;   // same as Turrettest, which was flipped to this on the robot
    public static final double INTAKE_SERVO_POWER = -1.0;  // from IntakeTeleOp's servo-fix PR; flip if it pushes balls out
    public static final double FEED_POWER         = -1.0;  // feeding motor while shooting

    /* ===================== FLYWHEEL ===================== */
    public static final double FLYWHEEL_DIRECTION     = -1.0;  // the flywheel shoots with NEGATIVE power on this robot
    public static final double FLYWHEEL_TICKS_PER_REV = 28.0;  // TODO confirm: encoder ticks per flywheel turn (bare goBILDA motor = 28)
    public static final double FLYWHEEL_RPM_TOLERANCE = 100;   // "at speed" = within this many RPM of the target
    public static final double AUTO_FLYWHEEL_RPM      = 2200;  // RPM the autos shoot at
    // Autos wait for the flywheel before Path 1: at least REV_MIN_MS, at most REV_TIMEOUT_MS.
    public static final double REV_MIN_MS     = 800;
    public static final double REV_TIMEOUT_MS = 3000;
    // TeleOp flywheel speed controller (Pedro PIDController) - TODO: tune.
    public static final double FLYWHEEL_P = 0.001, FLYWHEEL_I = 0, FLYWHEEL_D = 0;
    // FlywheelPIDF (Turret Test) also uses F: power per target RPM, about 1 / max RPM. Max RPM sets what 100% means.
    public static final double FLYWHEEL_F = 0.00017;  // TODO: tune
    public static final double FLYWHEEL_MAX_RPM = 6000; // TODO confirm: free speed of the flywheel motor (6000 for a bare goBILDA 5203)
    // TeleOp shot table: {distance from turret to CELL in inches, flywheel RPM, ramp/hood position (unused for now)}.
    // PLACEHOLDER rows - replace with tested values. Distances in between are blended.
    public static final double[][] SHOT_TABLE = {
        {24, 2400, 0.30},
        {60, 2900, 0.45},
        {96, 3400, 0.60},
    };

    /* ===================== DRIVER CONTROLS ===================== */
    public static final double TURN_SCALE     = 0.6;  // right stick turn speed (1.0 = full)
    public static final double STICK_DEADZONE = 0.1;  // stick movement smaller than this is ignored
}
