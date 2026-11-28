package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.RobotConfig.*;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

/**
 * Ramp Test - find the two ramp numbers for RobotConfig.
 *
 *   1. Power on the robot with the flower ramp resting where it normally starts (a bit up). Run this TeleOp.
 *   2. The servo does NOT move until you press something. Press D-pad UP or DOWN once; it then goes to the
 *      number shown (starts at 0.50), so keep the ramp clear in case it jumps.
 *   3. D-pad UP / DOWN  : move 0.01     LEFT / RIGHT bumper : move 0.05 (keep it small and slow!)
 *   4. Nudge until the ramp sits EXACTLY where it rests at power-on: note it as RAMP_UP_POS.
 *   5. Keep nudging until the flower is on the floor: note it as RAMP_DOWN_POS. Do not push into the floor.
 *   6. Type both numbers into RobotConfig.java.
 */
@TeleOp(name = "Ramp Test", group = "Test")
public class RampTest extends OpMode {
    private ServoImplEx ramp;
    private double pos = 0.5;
    private boolean moved = false;
    private boolean up, down, lb, rb; // button edges

    @Override
    public void init() {
        ramp = hardwareMap.get(ServoImplEx.class, RAMP_SERVO_NAME);
        ramp.setPwmRange(new PwmControl.PwmRange(500, 2500)); // not commanded yet: no jump
        telemetry.addLine("Ramp Test: press D-pad up/down to start. Servo stays limp until then.");
        telemetry.update();
    }

    @Override
    public void loop() {
        double step = 0;
        if (gamepad1.dpad_up && !up) step += 0.01;
        if (gamepad1.dpad_down && !down) step -= 0.01;
        if (gamepad1.right_bumper && !rb) step += 0.05;
        if (gamepad1.left_bumper && !lb) step -= 0.05;
        up = gamepad1.dpad_up; down = gamepad1.dpad_down; rb = gamepad1.right_bumper; lb = gamepad1.left_bumper;

        if (step != 0) {
            pos = Range.clip(pos + step, 0.0, 1.0);
            moved = true;
        }
        if (moved) ramp.setPosition(pos);

        telemetry.addData("RAMP SERVO POSITION", "%.3f %s", pos, moved ? "" : "(not commanded yet - press D-pad)");
        telemetry.addData("Servo reports (last commanded)", "%.3f", ramp.getPosition());
        telemetry.addData("Saved in RobotConfig", "up %.2f   down %.2f", RAMP_UP_POS, RAMP_DOWN_POS);
        telemetry.addLine("Write down the position at the power-on rest = UP, and on the floor = DOWN.");
        telemetry.update();
    }
}
