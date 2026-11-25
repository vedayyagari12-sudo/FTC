package org.firstinspires.ftc.teamcode;

/**
 * TurretAim - works out where the turret must point to shoot into the alliance's HIVE, from the robot pose.
 * Pure math (no hardware), so every auto can share it. See CODE_EXPLAINED.md.
 *
 * Frame = Pedro Pathing 3: origin bottom-left, inches, audience wall y = 0, red wall x = 0, blue wall x = 144,
 * heading counter-clockwise from +x (the same numbers the autos already use).
 *
 * The HIVE sits in the middle of the field (pivot at 72, 72). Each alliance's HIVE has a CELL at the audience end
 * and one at the far end. The robot aims at the CELL on the half of the field it is standing on:
 * y < 72 -> audience CELL, otherwise far CELL (the same rule as the Desmos simulation).
 */
public class TurretAim {
    public static final int RED = -1, BLUE = 1;

    /* ---------------- FIELD (Competition Manual TU06 sec 9) ---------------- */
    static final double HIVE_SEP = 12.75;  // red HIVE x = 72 - 12.75, blue HIVE x = 72 + 12.75
    static final double CELL_OFF = 16.5;   // CELL opening center is this far from y = 72 (derived - measure your field)
    static final double CELL_Z   = 58.3;   // CELL opening center height in inches (derived - measure your field)

    /* ---------------- ROBOT (PLACEHOLDERS - measure on the robot) ---------------- */
    // Turret removed for now (2026-11-25): the "pivot" is the fixed flywheel shaft, so SHAFT_AHEAD_IN stays 0.
    static final double PIVOT_FORWARD_IN = -5.5; // measured: flywheel shaft about 5.5 in BEHIND the robot center (negative = behind)
    static final double PIVOT_LEFT_IN    = 0;    // assumed centered side to side (negative = right); measure to confirm
    static final double SHOOTER_Z_IN     = 12; // TODO: height the ball leaves the shooter
    // The team measures shot distances from the flywheel shaft, so distanceIn is from the shaft too.
    static final double SHAFT_AHEAD_IN   = 0;  // TODO: flywheel shaft this far in FRONT of the turret pivot, along the way the shooter points (negative = behind)

    /* ---------------- TURRET SERVO (standard positional Servo, not continuous; Turrettest uses these too) ---------------- */
    static final double TURRET_CENTER    = 0.5;   // servo position that points the way the robot faces
    static final double TURRET_RANGE_DEG = 300.0; // TODO confirm: degrees of turret per full servo travel (goBILDA 2000 series = 300)
    static final double TURRET_LIMIT_DEG = 135.0; // turret can go this far to each side: 270 degrees of turning in total
    // +1: a LEFT (counter-clockwise) turn = higher servo position (what RedTeleOp assumes).
    // Turrettest's comment suggests the opposite, so test on the robot and flip to -1.0 if the turret turns the wrong way.
    static final double TURRET_SIGN      = 1.0;

    public static class Result {
        public boolean targetFar;    // false = audience CELL, true = far CELL
        public double targetX, targetY;
        public double turretDeg;     // turret angle from straight ahead, counter-clockwise positive, clamped to the limit
        public boolean outOfRange;   // the target needs more than TURRET_LIMIT_DEG, so turretDeg was clamped
        public double servoPos;      // value for turretServo.setPosition()
        public double distanceIn;    // flat distance from the flywheel shaft to the CELL opening (for the shot table)
        public double heightIn;      // CELL_Z - SHOOTER_Z_IN
    }

    /** x, y in inches and headingRad in radians straight from follower.pose(); alliance is RED or BLUE. */
    public static Result aim(double x, double y, double headingRad, int alliance) {
        return aim(x, y, headingRad, alliance, y >= 72);
    }

    /** Same, but the caller picks the CELL (RedTeleOp picks it from AprilTags or the D-pad). */
    public static Result aim(double x, double y, double headingRad, int alliance, boolean targetFar) {
        Result r = new Result();
        double px = x + PIVOT_FORWARD_IN * Math.cos(headingRad) - PIVOT_LEFT_IN * Math.sin(headingRad);
        double py = y + PIVOT_FORWARD_IN * Math.sin(headingRad) + PIVOT_LEFT_IN * Math.cos(headingRad);

        r.targetFar = targetFar;
        r.targetX = 72 + alliance * HIVE_SEP;
        r.targetY = 72 + (r.targetFar ? CELL_OFF : -CELL_OFF);

        double fieldDeg = Math.toDegrees(Math.atan2(r.targetY - py, r.targetX - px));
        double raw = wrap180(fieldDeg - Math.toDegrees(headingRad));
        r.turretDeg = Math.max(-TURRET_LIMIT_DEG, Math.min(TURRET_LIMIT_DEG, raw));
        r.outOfRange = Math.abs(raw) > TURRET_LIMIT_DEG;
        r.servoPos = TURRET_CENTER + TURRET_SIGN * r.turretDeg / TURRET_RANGE_DEG;
        // The shooter points at the CELL, so the shaft sits on the pivot-to-CELL line, SHAFT_AHEAD_IN closer.
        r.distanceIn = Math.hypot(r.targetX - px, r.targetY - py) - SHAFT_AHEAD_IN;
        r.heightIn = CELL_Z - SHOOTER_Z_IN;
        return r;
    }

    /** Wraps an angle to [-180, 180). */
    static double wrap180(double deg) {
        return ((deg + 180) % 360 + 360) % 360 - 180;
    }

    // Self-check: java TurretAim.java (numbers match the Desmos simulation's formulas).
    public static void main(String[] args) {
        Result r = aim(61, 34, Math.toRadians(90), RED);              // Close Red shooting spot
        check(!r.targetFar && near(r.targetX, 59.25, 1e-9) && near(r.targetY, 55.5, 1e-9), "close red target");
        // Expected values assume PIVOT_FORWARD_IN = -5.5, PIVOT_LEFT_IN = 0, SHAFT_AHEAD_IN = 0 (shooter 5.5 in behind center).
        check(near(r.turretDeg, 3.71, 0.01) && near(r.distanceIn, 27.06, 0.01) && near(r.servoPos, 0.5 + 3.7084 / 300, 1e-4), "close red aim");
        r = aim(83, 110, Math.toRadians(270), BLUE);                  // Close Blue shooting spot
        check(r.targetFar && near(r.targetX, 84.75, 1e-9) && near(r.targetY, 88.5, 1e-9) && near(r.turretDeg, 3.71, 0.01), "close blue aim");
        r = aim(57, 113, Math.toRadians(270), RED);                   // Far Red shooting spot, robot already faces the HIVE
        check(r.targetFar && !r.outOfRange, "far red in range");
        r = aim(60, 20, Math.toRadians(-90), RED);                    // facing away from the HIVE: needs about 180, so it clamps
        check(r.outOfRange && near(Math.abs(r.turretDeg), 135, 1e-9), "clamp");
        r = aim(61, 34, Math.toRadians(90), RED, true);               // forced far CELL from the lower half
        check(r.targetFar && near(r.targetY, 88.5, 1e-9), "forced target");
        check(near(wrap180(190), -170, 1e-9) && near(wrap180(-180), -180, 1e-9), "wrap");
        System.out.println("TurretAim self-check passed");
    }
    private static boolean near(double v, double want, double tol) { return Math.abs(v - want) <= tol; }
    private static void check(boolean ok, String name) { if (!ok) throw new AssertionError("TurretAim self-check failed: " + name); }
}
