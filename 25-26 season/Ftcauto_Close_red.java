package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.CRServo;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.VoltageSensor;

@Autonomous(name = "Ftcauto_Close_red")
public class Ftcauto_Close_red extends LinearOpMode {

    // ---------------- MOTORS ----------------
    DcMotor frontLeft, frontRight, backLeft, backRight;
    DcMotor intakemotor,ShootMotor;

    DcMotorEx flywheel;

    // ---------------- SERVOS ----------------
    CRServo IntakeServo1, IntakeServo2;
    Servo IntakeServo3;



    // ---------------- TIMERS ----------------
    ElapsedTime servoTimer = new ElapsedTime();
    ElapsedTime runtime = new ElapsedTime();
    ElapsedTime flyTelemetryTimer = new ElapsedTime(); // ADDED
    ElapsedTime flyTimer = new ElapsedTime();



    IMU imu;

    // ---------------- FLYWHEEL PID ----------------
    double kP = 0.00026666665;
    double kI = 0.0000004167;
    double kD = 0.00001;
    double kF = 0.80 / 3500.0;

    double flyIntegral = 0.0;
    double flyIntegralMax = 500;
    double flyLastError = 0.0;

    double rpmRaw = 0.0;
    double filteredRPM = 0.0;

    double lastTime = 0.0;
    double lastFly = 0.0;

    // double currentVoltage;

    int targetRPM = 0;
    double TRACK_WIDTH = 16.7;
    int shootticks= 0;
    static final double FLY_TICKS_PER_REV = 28.0;

    // ---------------- DRIVE CONSTANTS ----------------
    static final double TICKS_PER_REV = 537.6;
    static final double WHEEL_DIAMETER_IN = 4.25;
    static final double TICKS_PER_INCH =
            TICKS_PER_REV / (Math.PI * WHEEL_DIAMETER_IN);


    @Override
    public void runOpMode() {
        //
        //VoltageSensor vs = hardwareMap.voltageSensor.iterator().next();
        //currentVoltage = vs.getVoltage(); // FIX: no shadowing
        // ---------------- HARDWARE MAP ----------------
        frontLeft  = hardwareMap.get(DcMotor.class, "FrontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "FrontRight");
        backLeft   = hardwareMap.get(DcMotor.class, "BackLeft");
        backRight  = hardwareMap.get(DcMotor.class, "BackRight");

        intakemotor = hardwareMap.get(DcMotor.class, "intakemotor");
        flywheel    = hardwareMap.get(DcMotorEx.class, "flywheel");
        ShootMotor    = hardwareMap.get(DcMotor.class, "shootmotor");


        IntakeServo1 = hardwareMap.get(CRServo.class, "intakeservo");
        IntakeServo2 = hardwareMap.get(CRServo.class, "intakeservo2");
        IntakeServo3 = hardwareMap.get(Servo.class, "intakeservo3");

        // ---------------- MOTOR SETUP ----------------


        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        flywheel.setDirection(DcMotor.Direction.REVERSE);


        flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        ShootMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ShootMotor.setTargetPosition(0);
        ShootMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ShootMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ShootMotor.setPower(0);

        setBrake(true);
        resetEncoders();

        // ---------------- IMU INIT ----------------
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot orientation =
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                );
        imu.initialize(new IMU.Parameters(orientation));
        imu.resetYaw();

        telemetry.addLine("Auto Ready");
        telemetry.update();

        waitForStart();
        if (!opModeIsActive()) return;

        runtime.reset();
        flyTelemetryTimer.reset();

        flyTimer.reset();
        lastFly = flywheel.getCurrentPosition();
        filteredRPM = 0.0;
        flyIntegral = 0.0;
        flyLastError = 0.0;



// ---------------- AUTO PATH ----------------
        IntakeServo3.setPosition(0.3);
        sleep(200);
        targetRPM= 2290;

        driveForward(-34, 0.7);
        turnToAngle(-6.7, 0.7);
        ShootingLogic(3);
        sleep(500);//Drives back shoots and wait for 1 second to finish shooting


        targetRPM= 2400;//sets taget rpm higher because its farther than the other shooting point
        intakemotor.setPower(-1);
        turnToAngle(35, 0.7);
        strafeRight(21, 0.7);
        driveForward(34, 0.7);
        driveForward(-34, 0.7);
        turnToAngle(-42, 0.7);
        ShootingLogic(3);
        sleep(500); //Turns and strafes to the next rows of balls strafe and turn back and shoots then wait of for second to finish

        intakemotor.setPower(-1);
        turnToAngle(43, 0.7);
        strafeRight(22, 0.7);
        driveForward(40, 0.7);
        driveForward(-40, 0.7);
        strafeRight(-35,0.6);
        turnToAngle(-40,0.6);
        ShootingLogic(5);
        sleep(500);//turns back and strafes to collect more balls then goes back to the last shooting point to shoot them then wait 1 second to finish

        turnToAngle(45, 0.7);
        strafeRight(35, 0.7); //moves out of the white line




        stopMotors();




        // ---------------- FLYWHEEL PID RUN ----------------


        long start = System.currentTimeMillis();
        while (opModeIsActive() && System.currentTimeMillis() - start < 3000) {
            updateFlywheelPID(targetRPM);
            idle();
        }


        stopMotors();
        intakePower(0);
        flywheel.setPower(0);
    }

    // ---------------- FLYWHEEL PID METHOD ----------------
    void updateFlywheelPID(double targetRPM) {
        //       currentVoltage = hardwareMap.voltageSensor.iterator().next().getVoltage();

        // if (currentVoltage > 13.0)      kF = 1.0 / 4700.0;
        // else if (currentVoltage > 12.5) kF = 1.0 / 4500.0;
        // else                            kF = 1.0 / 4000.0;

        double dt = flyTimer.seconds();
        flyTimer.reset();

        if (dt <= 0) return;

        double currentPos = flywheel.getCurrentPosition();
        double deltaTicks = currentPos - lastFly;

        // RPM calculation (correct and stable)
        rpmRaw = (deltaTicks * 60.0) / (FLY_TICKS_PER_REV * dt);

        // Low-pass filter
        filteredRPM = 0.8 * filteredRPM + 0.2 * rpmRaw;

        lastFly = currentPos;

        // PID error
        double error = targetRPM - filteredRPM;

        // Integral (clamped)
        flyIntegral += error * dt;
        flyIntegral = Math.max(-flyIntegralMax,
                Math.min(flyIntegralMax, flyIntegral));

        // Derivative
        double derivative = (error - flyLastError) / dt;
        flyLastError = error;

        // PID + Feedforward
        double output =
                (kF * targetRPM) +
                        (kP * error) +
                        (kI * flyIntegral) +
                        (kD * derivative);

        // Apply power (motor reversed)
        flywheel.setPower(Math.max(0.0, Math.min(1.0, output)));

        // Telemetry at 5 Hz
        if (flyTelemetryTimer.seconds() >= 0.2) {
            // telemetry.addData("KF", kF);
            // telemetry.addData("Voltage", currentVoltage);
            telemetry.addData("Target RPM", targetRPM);
            telemetry.addData("Flywheel RPM", filteredRPM);
            telemetry.addData("Raw RPM", rpmRaw);
            telemetry.addData("Flywheel Power", flywheel.getPower());
            telemetry.update();
            flyTelemetryTimer.reset();
        }
    }



    // ---------------- SUBSYSTEM METHODS ----------------
    void intakePower(double power) {
        intakemotor.setPower(power);
    }

    // ---------------- SHOOTING ----------------
    public void ShootingLogic(int shots) {

        for (int i = 0; i <= shots && opModeIsActive(); i++) {
            updateFlywheelPID(targetRPM);



            // ---- START SHOT ----
            shootticks -=- 179;

            IntakeServo3.setPosition(1);      // push ring
            ShootMotor.setTargetPosition(shootticks);


            servoTimer.reset();

            if (i==0) {
                intakemotor.setPower(0);
                sleep(750);}

            ShootMotor.setPower(1.0);

            //idle();


            // ---- RETRACT ----

            sleep(500);


            //idle();

            ShootMotor.setPower(0.0);
            if (shots>=3) {
                intakemotor.setPower(-1);
            }


            if (shots<=1 || shots==i) {
                IntakeServo3.setPosition(0.3);
                intakemotor.setPower(0);
            }

            // Small pause between shots
            sleep(600);

        }
    }
    // ---------------- DRIVE METHODS ----------------
    void driveForward(double inches, double power) {
        int ticks = (int)(inches * TICKS_PER_INCH);
        setTarget(ticks, ticks, ticks, ticks);
        runToPosition(power);
    }

    void strafeRight(double inches, double power) {
        int ticks = (int)(inches * TICKS_PER_INCH );
        setTarget(ticks, -ticks, -ticks, ticks);
        runToPosition(power);
    }

    void turnToAngle(double targetDeg, double power) {
        double r = (targetDeg / 360.0) * Math.PI * TRACK_WIDTH*1.6;
        int turnTicks = (int) (r * TICKS_PER_INCH);
        setTarget(turnTicks,-turnTicks,turnTicks,-turnTicks);
        runToPosition(power);

    }


    // ---------------- HELPERS ----------------
    void setTarget(int fl, int fr, int bl, int br) {
        frontLeft.setTargetPosition(frontLeft.getCurrentPosition() + fl);
        frontRight.setTargetPosition(frontRight.getCurrentPosition() + fr);
        backLeft.setTargetPosition(backLeft.getCurrentPosition() + bl);
        backRight.setTargetPosition(backRight.getCurrentPosition() + br);
    }

    void runToPosition(double power) {
        setMode(DcMotor.RunMode.RUN_TO_POSITION);
        setPower(power, power, power, power);

        while (opModeIsActive()
                && frontLeft.isBusy()
                && frontRight.isBusy()
                && backLeft.isBusy()
                && backRight.isBusy()) {
            updateFlywheelPID(targetRPM);

            idle();
        }

        stopMotors();
        setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sleep(150);
    }

    void stopMotors() {
        setPower(0, 0, 0, 0);
    }

    void resetEncoders() {
        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    void setMode(DcMotor.RunMode mode) {
        frontLeft.setMode(mode);
        frontRight.setMode(mode);
        backLeft.setMode(mode);
        backRight.setMode(mode);
    }

    void setPower(double fl, double fr, double bl, double br) {
        frontLeft.setPower(fl);
        frontRight.setPower(fr);
        backLeft.setPower(bl);
        backRight.setPower(br);
    }

    void setBrake(boolean brake) {
        DcMotor.ZeroPowerBehavior z =
                brake ? DcMotor.ZeroPowerBehavior.BRAKE
                        : DcMotor.ZeroPowerBehavior.FLOAT;

        frontLeft.setZeroPowerBehavior(z);
        frontRight.setZeroPowerBehavior(z);
        backLeft.setZeroPowerBehavior(z);
        backRight.setZeroPowerBehavior(z);
    }
}
