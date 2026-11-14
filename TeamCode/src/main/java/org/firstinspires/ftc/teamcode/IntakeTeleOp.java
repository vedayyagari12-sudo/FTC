package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.RobotConfig.*;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "IntakeTeleOp")
public class IntakeTeleOp extends OpMode {

    /* ---------------- HARDWARE ---------------- */
    DcMotor FrontLeftWheel, FrontRightWheel, BackLeftWheel, BackRightWheel;
    DcMotor IntakeMotor, FeedingMotor;
    CRServo IntakeServo;

    /* Names, powers and TURN_SCALE are in RobotConfig.java. */

    /* ---------------- STATE ---------------- */
    boolean intakeOn = false;
    boolean intakeToggleReady = true;

    /* ---------------- INIT ---------------- */
    @Override
    public void init() {
        FrontLeftWheel  = hardwareMap.dcMotor.get(FRONT_LEFT_NAME);
        FrontRightWheel = hardwareMap.dcMotor.get(FRONT_RIGHT_NAME);
        BackLeftWheel   = hardwareMap.dcMotor.get(BACK_LEFT_NAME);
        BackRightWheel  = hardwareMap.dcMotor.get(BACK_RIGHT_NAME);

        IntakeMotor  = hardwareMap.dcMotor.get(INTAKE_MOTOR_NAME);
        IntakeServo  = hardwareMap.get(CRServo.class, INTAKE_SERVO_NAME);
        FeedingMotor = hardwareMap.dcMotor.get(FEEDING_MOTOR_NAME);

        BackLeftWheel.setDirection(DcMotor.Direction.REVERSE);
        FrontLeftWheel.setDirection(DcMotor.Direction.REVERSE);

        FrontLeftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontRightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackLeftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        IntakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        IntakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        FeedingMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FeedingMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FeedingMotor.setPower(0);

        stopIntake();

        telemetry.addLine("Initialized. B toggles intake, hold right trigger to feed.");
        telemetry.update();
    }

    /* ---------------- LOOP ---------------- */
    @Override
    public void loop() {

        /* -------- MECANUM DRIVE (gamepad1) -------- */
        double y  = -gamepad1.left_stick_y;
        double x  =  gamepad1.left_stick_x;
        double rx =  gamepad1.right_stick_x * TURN_SCALE;

        double den = Math.max(1.0, Math.abs(y) + Math.abs(x) + Math.abs(rx));

        FrontLeftWheel.setPower((y + x + rx) / den);
        FrontRightWheel.setPower((y - x - rx) / den);
        BackLeftWheel.setPower((y - x + rx) / den);
        BackRightWheel.setPower((y + x - rx) / den);

        /* -------- INTAKE TOGGLE (gamepad2 B) -------- */
        if (!gamepad2.b) intakeToggleReady = true;

        if (gamepad2.b && intakeToggleReady) {
            intakeOn = !intakeOn;
            intakeToggleReady = false;
        }

        if (intakeOn) runIntake();
        else          stopIntake();

        /* -------- FEED (gamepad2 right trigger, hold) -------- */
        FeedingMotor.setPower(gamepad2.right_trigger > 0.5 ? FEED_POWER : 0);

        telemetry.addData("Intake", intakeOn ? "RUNNING" : "STOPPED");
        telemetry.update();
    }

    /* ---------------- HELPERS ---------------- */
    private void runIntake() {
        IntakeMotor.setPower(INTAKE_MOTOR_POWER);
        IntakeServo.setPower(INTAKE_SERVO_POWER);
    }

    private void stopIntake() {
        IntakeMotor.setPower(0);
        IntakeServo.setPower(0);
    }

    @Override
    public void stop() {
        stopIntake();
        FeedingMotor.setPower(0);
        FrontLeftWheel.setPower(0);
        FrontRightWheel.setPower(0);
        BackLeftWheel.setPower(0);
        BackRightWheel.setPower(0);
    }
}
