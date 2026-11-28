package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.RobotConfig.*;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * RampControl - moves the flower ramp ("rampServo") between its default UP position and the floor.
 *
 * rampServo is a normal POSITION servo (setPosition 0 to 1), NOT a continuous-rotation servo, so the code uses
 * Servo (not CRServo). The two ramp servos share one signal wire, so one Servo object moves both.
 * A position servo cannot report where it is, so the code keeps track of the position it last commanded.
 * The ramp moves smoothly (RAMP_SPEED per second) and never goes past RAMP_DOWN_POS, so it stops at the floor.
 *
 * USE IN AN OPMODE:
 *   init():  ramp = new RampControl(hardwareMap);   // does NOT move the servo
 *   loop():  ramp.update();                          // every loop
 *            ramp.lower();                           // ask for the floor (keep calling it every loop while you want it down)
 *            ramp.raise();                           // ask for the default UP position
 *
 * TeleOp: if (gamepad2.left_trigger > RAMP_TRIGGER) ramp.lower(); else ramp.raise();   -> hold to lower, release to reset
 * Autos : ramp.lower() when the robot reaches the flower, ramp.raise() when the intake is done.
 *
 * Until RAMP_UP_POS and RAMP_DOWN_POS are different numbers in RobotConfig the ramp is never moved.
 */
public class RampControl {
    private final ServoImplEx servo;
    private final ElapsedTime timer = new ElapsedTime();
    private double pos = RAMP_UP_POS;      // where we think the ramp is (no sensor: it rests UP at power-on)
    private double target = RAMP_UP_POS;
    private boolean active = false;        // becomes true at the first lower(); before that the servo is never commanded
    private double lastMs = 0;

    public RampControl(HardwareMap hardwareMap) {
        servo = hardwareMap.get(ServoImplEx.class, RAMP_SERVO_NAME);
        servo.setPwmRange(new PwmControl.PwmRange(500, 2500));   // goBILDA full range: 0.0 now goes further "up" than the default 600-2400
        // Not commanded here on purpose: commanding a position at INIT would make the ramp jump if it is wrong.
    }

    /** Go to the floor. Safe to call every loop. */
    public void lower() {
        if (RAMP_UP_POS == RAMP_DOWN_POS) return;   // positions not measured yet
        target = RAMP_DOWN_POS;
        active = true;
    }

    /** Go back to the default UP position. Safe to call every loop. Does nothing if the ramp never went down. */
    public void raise() {
        target = RAMP_UP_POS;
    }

    /** Call every loop. */
    public void update() {
        double now = timer.milliseconds();
        double dt = (now - lastMs) / 1000.0;
        lastMs = now;
        if (!active) return;
        double step = RAMP_SPEED * dt;
        if (pos < target) pos = Math.min(target, pos + step);
        else if (pos > target) pos = Math.max(target, pos - step);
        // Hard stop: never past the floor or past the up position.
        pos = Math.max(Math.min(RAMP_UP_POS, RAMP_DOWN_POS), Math.min(Math.max(RAMP_UP_POS, RAMP_DOWN_POS), pos));
        servo.setPosition(pos);
    }

    public boolean isDown() {
        return active && pos == RAMP_DOWN_POS;
    }

    public String status() {
        if (RAMP_UP_POS == RAMP_DOWN_POS) return "not set up (RAMP_UP_POS = RAMP_DOWN_POS in RobotConfig)";
        return String.format("%s, position %.2f", target == RAMP_DOWN_POS ? "going down" : "up", pos);
    }
}
