package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * FlowerRamp - the flower ramp code in one file (same logic as RampControl + the ramp numbers from RobotConfig).
 * Drop this file into TeamCode next to the other OpModes. It does not depend on RobotConfig.
 *
 * Hardware: "rampServo", Expansion Hub servo port 0, standard POSITION servo (not CRServo).
 *           The two ramp servos share one signal wire, so one Servo object moves both.
 * Uses the wide goBILDA pulse range 500-2500 us, so the positions below are for that range.
 *
 * ---------------- HOW TO USE ----------------
 *   Field:   FlowerRamp ramp;
 *   init():  ramp = new FlowerRamp(hardwareMap);   // does NOT move the servo (no jump at init)
 *   loop():  ramp.update();                         // every loop
 *
 *   TeleOp (hold gamepad 2 left trigger to lower, let go to raise):
 *            if (gamepad2.left_trigger > FlowerRamp.TRIGGER) ramp.lower(); else ramp.raise();
 *            ramp.update();
 *            telemetry.addData("Ramp", ramp.status());
 *
 *   Autos:   ramp.lower();  when the robot reaches the flower (when the intake turns on)
 *            ramp.raise();  when the intake is done
 *            ramp.update(); every loop
 *
 * Until UP_POS and DOWN_POS are different numbers the ramp is never moved.
 * Find them with the "Ramp Test" TeleOp (D-pad up/down = 0.01, bumpers = 0.05).
 */
public class FlowerRamp {

    /* ---------------- RAMP NUMBERS (tune these) ---------------- */
    public static final String SERVO_NAME = "rampServo";
    public static final double DOWN_POS   = 0.583; // on the floor
    public static final double UP_POS     = 0.0;   // default up position (highest spot where it still moves freely)
    // After touching the floor the ramp backs off by this much (toward UP_POS) and rides there, about 1 in above the ground.
    // TODO tune: roughly DOWN_POS / (inches the ramp tip is above the floor at UP_POS). 10 in up -> 0.058. Raise it to ride higher.
    public static final double HOVER_POS  = 0.06;
    public static final double SPEED      = 1.0;   // servo position units per second (bigger = faster)
    public static final double TRIGGER    = 0.5;   // left trigger must be pushed past this to lower the ramp

    private final ServoImplEx servo;
    private final ElapsedTime timer = new ElapsedTime();
    private double pos = UP_POS;     // where we think the ramp is (no sensor: it rests UP at power-on)
    private double target = UP_POS;
    private boolean active = false;  // true after the first lower(); before that the servo is never commanded
    private boolean lowering = false; // true from lower() until raise(): down to the floor, then back off to the hover spot
    private double lastMs = 0;

    public FlowerRamp(HardwareMap hardwareMap) {
        servo = hardwareMap.get(ServoImplEx.class, SERVO_NAME);
        servo.setPwmRange(new PwmControl.PwmRange(500, 2500)); // goBILDA full range
        // Not commanded here on purpose, so the ramp doesn't jump at INIT.
    }

    /** Go all the way down to the floor, then back up a little to the hover spot. Safe to call every loop. */
    public void lower() {
        if (UP_POS == DOWN_POS) return; // positions not measured yet
        if (!lowering) { // only the first call starts the move, so calling it every loop does not restart it
            lowering = true;
            target = DOWN_POS;
        }
        active = true;
    }

    /** Go back up. Safe to call every loop. Does nothing if the ramp never went down. */
    public void raise() {
        lowering = false;
        target = UP_POS;
    }

    /** Call every loop: moves the ramp smoothly toward the target. */
    public void update() {
        double now = timer.milliseconds();
        double dt = (now - lastMs) / 1000.0;
        lastMs = now;
        if (!active) return;
        double step = SPEED * dt;
        if (pos < target) pos = Math.min(target, pos + step);
        else if (pos > target) pos = Math.max(target, pos - step);
        // Hard stop: never past the floor or past the up position.
        pos = Math.max(Math.min(UP_POS, DOWN_POS), Math.min(Math.max(UP_POS, DOWN_POS), pos));
        if (lowering && pos == DOWN_POS) target = hoverPos(); // touched the floor: now back off to the hover spot
        servo.setPosition(pos);
    }

    /** The spot about 1 in above the ground: DOWN_POS backed off by HOVER_POS toward UP_POS (never past UP_POS). */
    private static double hoverPos() {
        double back = Math.min(HOVER_POS, Math.abs(UP_POS - DOWN_POS));
        return DOWN_POS + Math.signum(UP_POS - DOWN_POS) * back;
    }

    /** True once the ramp has touched the floor and settled at the hover spot. */
    public boolean isDown() {
        return active && lowering && pos == hoverPos();
    }

    public String status() {
        if (UP_POS == DOWN_POS) return "not set up (UP_POS = DOWN_POS)";
        String state = !lowering ? "up" : pos == hoverPos() ? "down, hovering" : "going down";
        return String.format("%s, position %.2f", state, pos);
    }
}
