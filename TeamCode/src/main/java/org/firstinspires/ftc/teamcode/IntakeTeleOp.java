package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "IntakeTeleOp")
public class IntakeTeleOp extends OpMode {

    /* ---------------- HARDWARE ---------------- */
    DcMotor FrontLeftWheel, FrontRightWheel, BackLeftWheel, BackRightWheel;
    DcMotor IntakeMotor, ShootMotor;
    CRServo IntakeServo1, IntakeServo2;
    Servo IntakeServo3;

    /* ---------------- TUNING ---------------- */
    // Flip the sign on any of these if a piece runs backwards.
    final double INTAKE_MOTOR_POWER  = -1.0;
    final double INTAKE_SERVO1_POWER =  -1.0;
    final double INTAKE_SERVO2_POWER = 1.0;   // opposite side, so opposite sign
    final double TURN_SCALE = 0.6;

    // Ram/push mechanism (ShootMotor + IntakeServo3)
    final int    RAM_TICKS_PER_PRESS = 179;
    final double RAM_SERVO_EXTENDED  = 1.0;
    final double RAM_SERVO_RETRACTED = 0.3;
    final double RAM_MOTOR_POWER     = 1.0;
    final double RAM_POWER_DELAY_SEC = 0.5;   // wait before driving ShootMotor
    final double RAM_RETRACT_SEC     = 1.0;   // when to retract servo & cut power

    /* ---------------- STATE ---------------- */
    boolean intakeOn = false;
    boolean intakeToggleReady = true;

    int shootTicks = 0;
    boolean triggerReleased = true;
    ElapsedTime servoTimer = new ElapsedTime();

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

        ShootMotor   = hardwareMap.dcMotor.get("shootmotor");
        IntakeServo3 = hardwareMap.get(Servo.class, "intakeservo3");

        BackLeftWheel.setDirection(DcMotor.Direction.REVERSE);
        FrontLeftWheel.setDirection(DcMotor.Direction.REVERSE);

        FrontLeftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontRightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackLeftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        IntakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        IntakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        ShootMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ShootMotor.setTargetPosition(0);
        ShootMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ShootMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ShootMotor.setPower(0);

        IntakeServo3.setPosition(RAM_SERVO_RETRACTED);

        stopIntake();

        telemetry.addLine("Initialized. B toggles intake, right trigger fires ram.");
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

        /* -------- RAM / PUSH (gamepad2 right trigger) -------- */
        double trigger = gamepad2.right_trigger;

        if (trigger < 0.05) {
            triggerReleased = true;
        }

        if (trigger > 0.1 && triggerReleased) {
            shootTicks += RAM_TICKS_PER_PRESS;
            triggerReleased = false;

            IntakeServo3.setPosition(RAM_SERVO_EXTENDED);
            ShootMotor.setTargetPosition(shootTicks);

            servoTimer.reset();
        }

        if (servoTimer.seconds() >= RAM_POWER_DELAY_SEC) {
            ShootMotor.setPower(RAM_MOTOR_POWER);

            if (servoTimer.seconds() >= RAM_RETRACT_SEC) {
                IntakeServo3.setPosition(RAM_SERVO_RETRACTED);
                ShootMotor.setPower(0.0);
            }
        }

        telemetry.addData("Intake", intakeOn ? "RUNNING" : "STOPPED");
        telemetry.addData("Ram target ticks", shootTicks);
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
        ShootMotor.setPower(0);
        FrontLeftWheel.setPower(0);
        FrontRightWheel.setPower(0);
        BackLeftWheel.setPower(0);
        BackRightWheel.setPower(0);
    }
}
