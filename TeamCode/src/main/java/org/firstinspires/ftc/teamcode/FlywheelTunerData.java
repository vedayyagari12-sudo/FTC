package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.RobotConfig.*;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

import java.util.ArrayList;
import java.util.List;

/**
 * FlywheelTunerData - measures the numbers the Flywheel PIDF Tuner page needs
 * (~/Desktop/BioBuzz-Turret/flywheel-pidf-tuner.html). The robot does not drive.
 *
 * CONTROLS (gamepad 2)
 *   A             : test 1, spin-up. Unload all balls first. The flywheel must be stopped.
 *                   Runs full power for SPINUP_SECONDS, then shows K, tau, encoder noise and loop time.
 *   D-pad up/down : power for test 2 (+5% / -5%). Pick the power you shoot with.
 *   B             : test 2, ball dip. Spins at the test 2 power. When telemetry says READY,
 *                   hold RIGHT TRIGGER all the way down to feed one ball.
 *   X             : stop everything
 *
 * Both tests run the flywheel at a fixed power with no PID, which is what the tuner's model needs.
 */
@TeleOp(name = "Flywheel Tuner Data", group = "Test")
public class FlywheelTunerData extends OpMode {

    static final double SPINUP_SECONDS = 8.0;     // test 1 length, long enough for tau up to about 1.3 s. Raise it if telemetry says RPM was still climbing.
    static final double STEADY_SECONDS = 1.0;     // the end of test 1 used for K and noise
    static final double SETTLE_SECONDS = 3.0;     // test 2 waits this long before you may feed a ball
    static final double DIP_WINDOW_SECONDS = 1.0; // test 2 looks for the lowest RPM this long after the trigger
    static final double STOPPED_RPM = 50;         // test 1 only starts below this speed
    static final double TRIGGER_THRESHOLD = 0.9;

    enum State { IDLE, SPINUP, SETTLING, READY, FEEDING }

    DcMotorEx flywheel;
    DcMotor intakeMotor;
    DcMotor feedingMotor;
    FlywheelPIDF flywheelPIDF; // only used to read RPM

    State state = State.IDLE;
    double stateStart;
    double shotPower = 0.5;
    boolean aReady = true, bReady = true, dpadReady = true;
    String message = "Press A for test 1.";

    final List<double[]> samples = new ArrayList<>(); // {seconds since start, RPM}
    double[] spinup;       // {K, tau, noise, loopMs, stillClimbing 0/1}, null until test 1 runs
    double dipRpm = -1, dipAtRpm;
    double preShotRpm, lowestRpm;

    @Override
    public void init() {
        flywheel = hardwareMap.get(DcMotorEx.class, FLYWHEEL_NAME);
        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        // Same as Turret Test: reverse the motor so the shooting direction is positive power and positive RPM.
        flywheel.setDirection(FLYWHEEL_DIRECTION < 0 ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);
        flywheel.setPower(0);
        flywheelPIDF = new FlywheelPIDF(flywheel);

        intakeMotor = hardwareMap.dcMotor.get(INTAKE_MOTOR_NAME);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setPower(0);

        feedingMotor = hardwareMap.dcMotor.get(FEEDING_MOTOR_NAME);
        feedingMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        feedingMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        feedingMotor.setPower(0);

        telemetry.addLine("Flywheel Tuner Data ready. Unload all balls, then press START and A.");
        telemetry.update();
    }

    @Override
    public void loop() {
        double now = getRuntime();
        double rpm = flywheelPIDF.rpm();
        boolean trigger = gamepad2.right_trigger > TRIGGER_THRESHOLD;

        boolean aPressed = gamepad2.a && aReady;
        aReady = !gamepad2.a;
        boolean bPressed = gamepad2.b && bReady;
        bReady = !gamepad2.b;
        boolean dpadHeld = gamepad2.dpad_up || gamepad2.dpad_down;
        if (dpadHeld && dpadReady) shotPower = Range.clip(shotPower + (gamepad2.dpad_up ? 0.05 : -0.05), 0.05, 1.0);
        dpadReady = !dpadHeld;

        if (gamepad2.x) {
            state = State.IDLE;
            message = "Stopped.";
        }

        switch (state) {
            case IDLE:
                if (aPressed) {
                    if (rpm < STOPPED_RPM) {
                        samples.clear();
                        start(State.SPINUP, now);
                    } else {
                        message = "Wait for the flywheel to stop, then press A again.";
                    }
                } else if (bPressed) {
                    start(State.SETTLING, now);
                }
                break;
            case SPINUP:
                samples.add(new double[] {now - stateStart, rpm});
                if (now - stateStart >= SPINUP_SECONDS) {
                    spinup = analyzeSpinup(samples, STEADY_SECONDS);
                    message = "Test 1 done. Press B for test 2.";
                    state = State.IDLE;
                }
                break;
            case SETTLING:
                if (now - stateStart >= SETTLE_SECONDS) {
                    samples.clear();
                    start(State.READY, now);
                }
                break;
            case READY:
                // Keep the last 0.5 s so the speed before the shot is an average, not one noisy reading.
                samples.add(new double[] {now, rpm});
                while (now - samples.get(0)[0] > 0.5) samples.remove(0);
                if (trigger) {
                    preShotRpm = mean(samples);
                    lowestRpm = rpm;
                    start(State.FEEDING, now);
                }
                break;
            case FEEDING:
                lowestRpm = Math.min(lowestRpm, rpm);
                if (now - stateStart >= DIP_WINDOW_SECONDS) {
                    dipRpm = preShotRpm - lowestRpm;
                    dipAtRpm = preShotRpm;
                    message = "Test 2 done. Press B to shoot another ball.";
                    state = State.IDLE;
                }
                break;
        }

        boolean spinning = state == State.SETTLING || state == State.READY || state == State.FEEDING;
        flywheel.setPower(state == State.SPINUP ? 1.0 : spinning ? shotPower : 0);
        boolean feeding = state == State.FEEDING && trigger;
        feedingMotor.setPower(feeding ? FEED_POWER : 0);
        intakeMotor.setPower(feeding ? INTAKE_MOTOR_POWER : 0);

        /* -------- TELEMETRY -------- */
        telemetry.addData("Step", state == State.READY ? "READY: hold right trigger to feed one ball" : state.toString());
        telemetry.addLine(message);
        telemetry.addData("Flywheel RPM", "%.0f", rpm);
        telemetry.addData("Test 2 power (D-pad)", "%.0f%%", shotPower * 100);
        telemetry.addLine("--- Type these into the tuner page ---");
        if (spinup != null) {
            telemetry.addData("K, RPM at full power", "%.0f", spinup[0]);
            telemetry.addData("tau, seconds", "%.2f", spinup[1]);
            telemetry.addData("Encoder noise, +/-RPM", "%.0f", spinup[2]);
            telemetry.addData("Loop time, ms", "%.0f (this OpMode; Turret Test runs Pedro too and may be slower)", spinup[3]);
            if (spinup[4] > 0) telemetry.addLine("RPM was still climbing at the end: raise SPINUP_SECONDS and rerun test 1.");
        } else {
            telemetry.addLine("K, tau, noise, loop time: run test 1 (A)");
        }
        if (dipRpm >= 0) {
            telemetry.addData("Ball dip, RPM", "%.0f (from %.0f RPM)", dipRpm, dipAtRpm);
            telemetry.addData("Target RPM", "%.0f (the speed test 2 shot at)", dipAtRpm);
        } else {
            telemetry.addLine("Ball dip: run test 2 (B)");
        }
        telemetry.update();
    }

    void start(State next, double now) {
        state = next;
        stateStart = now;
        message = "";
    }

    static double mean(List<double[]> s) {
        double sum = 0;
        for (double[] p : s) sum += p[1];
        return sum / s.size();
    }

    /**
     * samples = {seconds since full power started, RPM}, starting from rest.
     * Returns {K, tau, noise, loopMs, stillClimbing}: K = mean RPM over the last steadySeconds,
     * tau = first time the RPM reaches 63.2% of K, noise = largest distance from K in that window,
     * stillClimbing = 1 if K is more than 2% above the window before it.
     */
    static double[] analyzeSpinup(List<double[]> samples, double steadySeconds) {
        double end = samples.get(samples.size() - 1)[0];
        double sum = 0, earlierSum = 0;
        int count = 0, earlierCount = 0;
        for (double[] p : samples) {
            if (p[0] >= end - steadySeconds) { sum += p[1]; count++; }
            else if (p[0] >= end - 2 * steadySeconds) { earlierSum += p[1]; earlierCount++; }
        }
        double k = sum / count;
        double noise = 0;
        for (double[] p : samples) if (p[0] >= end - steadySeconds) noise = Math.max(noise, Math.abs(p[1] - k));

        double tau = end;
        for (int i = 1; i < samples.size(); i++) {
            double[] a = samples.get(i - 1), b = samples.get(i);
            if (b[1] >= 0.632 * k) {
                // Straight line between the two readings around the 63.2% crossing.
                tau = b[1] == a[1] ? b[0] : a[0] + (0.632 * k - a[1]) / (b[1] - a[1]) * (b[0] - a[0]);
                break;
            }
        }
        double loopMs = end / (samples.size() - 1) * 1000;
        boolean climbing = earlierCount > 0 && k > 1.02 * (earlierSum / earlierCount);
        return new double[] {k, tau, noise, loopMs, climbing ? 1 : 0};
    }

    @Override
    public void stop() {
        if (flywheel != null) flywheel.setPower(0);
        if (intakeMotor != null) intakeMotor.setPower(0);
        if (feedingMotor != null) feedingMotor.setPower(0);
    }
}
