package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.RobotConfig.*;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

/**
 * Flywheel speed controller: power = kF * target + kP * error + kI * integral(error) + kD * d(error)/dt.
 * Speeds are in RPM. Call pidf(targetRpm) once per loop and give the result to motor.setPower().
 * The motor must spin forward with positive power (reverse it in init if the flywheel shoots the other way).
 *
 * The target is just a number you pass in: the Turret Test passes the D-pad speed, and later odometry can pass
 * the RPM looked up from the distance to the CELL (see SHOT_TABLE in RobotConfig).
 *
 * Tuning: set kP/kI/kD to 0, raise kF until the flywheel gets close to the target, then add kP until it settles
 * quickly, then a little kD or kI only if you need them.
 */
public class FlywheelPIDF {
    public double kP = FLYWHEEL_P, kI = FLYWHEEL_I, kD = FLYWHEEL_D, kF = FLYWHEEL_F;

    private final DcMotorEx motor;
    private double integral, lastError;
    private long lastNanos;
    private boolean running;

    public FlywheelPIDF(DcMotorEx motor) {
        this.motor = motor;
    }

    /** Current flywheel speed in RPM. */
    public double rpm() {
        return motor.getVelocity() / FLYWHEEL_TICKS_PER_REV * 60.0;
    }

    /** Motor power (0 to 1) that moves the flywheel toward targetRpm. A target of 0 or less returns 0 and resets. */
    public double pidf(double targetRpm) {
        if (targetRpm <= 0) {
            running = false;
            return 0;
        }
        long now = System.nanoTime();
        double dt = running ? (now - lastNanos) / 1e9 : 0;
        lastNanos = now;
        double power = calculate(targetRpm, rpm(), dt);
        running = true;
        return power;
    }

    // dt = 0 on the first loop after the flywheel starts: no integral or derivative yet.
    double calculate(double targetRpm, double currentRpm, double dt) {
        double error = targetRpm - currentRpm;
        double derivative = 0;
        if (dt > 0) {
            double limit = kI > 0 ? 1.0 / kI : 0; // keep kI * integral inside the power range (anti-windup)
            integral = Range.clip(integral + error * dt, -limit, limit);
            derivative = (error - lastError) / dt;
        } else {
            integral = 0;
        }
        lastError = error;
        return Range.clip(kF * targetRpm + kP * error + kI * integral + kD * derivative, 0, 1);
    }
}
