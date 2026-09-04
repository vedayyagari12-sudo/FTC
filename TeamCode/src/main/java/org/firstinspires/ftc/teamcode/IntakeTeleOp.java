package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "IntakeTeleOp")
public class IntakeTeleOp extends OpMode {
//code change
    /* ---------------- HARDWARE ---------------- */
    DcMotor FrontLeftWheel, FrontRightWheel, BackLeftWheel, BackRightWheel;
    DcMotor IntakeMotor;
    CRServo IntakeServo1, IntakeServo2;

    /* ---------------- TUNING ---------------- */
    // Flip the sign on any of these if a piece runs backwards.
    final double INTAKE_MOTOR_POWER  = 1.0;
    final double INTAKE_SERVO1_POWER =  1.0;
    final double INTAKE_SERVO2_POWER = -1.0;   // opposite side, so opposite sign
    final double TURN_SCALE = 0.6;

    /* ---------------- STATE ---------------- */
    boolean intakeOn = false;
    boolean intakeToggleReady = true;

    /* ---------------- INIT ---------------- */
    @Override
    public void init() {
        FrontLeftWheel  = hardwareMap.dcMotor.get("FrontLeft");
        FrontRightWheel = hardwareMap.dcMotor.get("FrontRight");
        BackLeftWheel   = hardwareMap.dcMotor.get("BackLeft");
        BackRightWheel  = hardwareMap.dcMotor.get("BackRight");

        IntakeMotor  = hardwareMap.dcMotor.get("intakemotor");
        IntakeServo1 = hardwareMap.get(CRServo.class, "intakeservo");
        IntakeServo2 = hardwareMap.get(CRServo.class, "intakeservo2");

        // Removed the reverse for the FrontLeftWheel and reversed the FrontRightWheel becuase of minor gear.
        BackLeftWheel.setDirection(DcMotor.Direction.REVERSE);
        FrontRightWheel.setDirection(DcMotor.Direction.REVERSE);

        FrontLeftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontRightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackLeftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        IntakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        IntakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        stopIntake();

        telemetry.addLine("Initialized. B toggles intake.");
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

        telemetry.addData("Intake", intakeOn ? "RUNNING" : "STOPPED");
        telemetry.update();
    }

    /* ---------------- HELPERS ---------------- */
    private void runIntake() {
        IntakeMotor.setPower(INTAKE_MOTOR_POWER);
        IntakeServo1.setPower(INTAKE_SERVO1_POWER);
        IntakeServo2.setPower(INTAKE_SERVO2_POWER);
    }

    private void stopIntake() {
        IntakeMotor.setPower(0);
        IntakeServo1.setPower(0);
        IntakeServo2.setPower(0);
    }

    @Override
    public void stop() {
        stopIntake();
        FrontLeftWheel.setPower(0);
        FrontRightWheel.setPower(0);
        BackLeftWheel.setPower(0);
        BackRightWheel.setPower(0);
    }
}
