package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.*;

@TeleOp(name = "ftcRedteleop")
public class ftcRedteleop extends OpMode {

    /* ---------------- HARDWARE ---------------- */
    DcMotor FrontLeftWheel, FrontRightWheel, BackLeftWheel, BackRightWheel;
    DcMotor IntakeMotor, FlyWheel, LiftMotor,ShootMotor;
    CRServo IntakeServo1, IntakeServo2;
    Servo IntakeServo3;

    /* ---------------- STATE ---------------- */
    boolean intakeOn = false;
    boolean flywheelOn = false;
    
    boolean liftUp = false;
    boolean liftToggleReady = true;
    boolean intakeToggleReady = true;
    boolean flyToggleReady = true;
   
    int shootticks = -179;

    
    boolean camToggleReady = true;
    boolean rpm2000Ready = true;
    boolean rpm2200Ready = true;
    boolean rpm2400Ready = true;
    boolean rpm2800Ready = true;

   
    /* ---------------- SERVO SYSTEM ---------------- */
    int ramState = 2;

    boolean triggerReleased = true;


    final double SERVO_MOVE_TIME = 1.5;
    ElapsedTime servoTimer = new ElapsedTime();
    ElapsedTime runtime = new ElapsedTime();

    /* ---------------- FLYWHEEL PID ---------------- */
    double currentVoltage;
    double targetFlyRPM  = 2000;

    double kP = 0.00026666665;
    double kI = 0.0000004167;
    double kD = 0.00002;
    double kF = 0.80 / 3600.0;

    double flyIntegral = 0;
    double flyLastError = 0;
    double flyIntegralMax = 150;

    double lastTime = 0;
    int lastFly = 0;
    double filteredRPM = 0;
    double rpmRaw = 0;

    final double MAX_FLY_POWER = 1.0;
    final double TICKS_PER_REV = 28.0;

    /* ---------------- APRILTAG ---------------- */
    VisionPortal visionPortal;
    AprilTagProcessor aprilTag;

    static final int TARGET_TAG_ID = 24;

    boolean tagVisible = false;
    double tagDistanceInches = -1;  
    double tagYaw = 0;
   



    double quadA = -0.0000573875;
    double quadB = 0.0192938;
    double quadC = -2.2134;
    double quadD = 115.45262;
    double quadE = 0;

    double minFlyRPM = 1500;
    double maxFlyRPM = 5600;

    boolean camtargeton = true;
    boolean flyspeed2000 = false;
    boolean flyspeed2200 = false;
    boolean flyspeed2400 = false;
    boolean flyspeed2800 = false;
    void updateFlywheelRPMFromAprilTag() {
        double rpm =
                quadA * Math.pow(tagDistanceInches, 4) +
                quadB * Math.pow(tagDistanceInches, 3) +
                quadC * Math.pow(tagDistanceInches, 2) +
                quadD * tagDistanceInches +
                quadE;

        targetFlyRPM = Math.max(minFlyRPM, Math.min(maxFlyRPM, rpm));
    }


    /* ---------------- INIT ---------------- */
    @Override
    public void init() {

        VoltageSensor vs = hardwareMap.voltageSensor.iterator().next();
        currentVoltage = vs.getVoltage(); // FIX: no shadowing

        FrontLeftWheel  = hardwareMap.dcMotor.get("FrontLeft");
        FrontRightWheel = hardwareMap.dcMotor.get("FrontRight");
        BackLeftWheel   = hardwareMap.dcMotor.get("BackLeft");
        BackRightWheel  = hardwareMap.dcMotor.get("BackRight");

        IntakeMotor = hardwareMap.dcMotor.get("intakemotor");
        FlyWheel    = hardwareMap.dcMotor.get("flywheel");
        LiftMotor   = hardwareMap.dcMotor.get("liftmotor");
        ShootMotor   = hardwareMap.dcMotor.get("shootmotor");

        IntakeServo1 = hardwareMap.get(CRServo.class, "intakeservo");
        IntakeServo2 = hardwareMap.get(CRServo.class, "intakeservo2");
        IntakeServo3 = hardwareMap.get(Servo.class, "intakeservo3");

        double fx = 822.317;
        double fy = 822.317; 
        double cx = 640.0; 
        double cy = 360.0;

        FrontLeftWheel.setDirection(DcMotor.Direction.REVERSE);
        BackLeftWheel.setDirection(DcMotor.Direction.REVERSE);
        FlyWheel.setDirection(DcMotor.Direction.REVERSE);
       

        FlyWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FlyWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LiftMotor.setTargetPosition(0);
        LiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LiftMotor.setPower(0);
        ShootMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ShootMotor.setTargetPosition(0);
        ShootMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ShootMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ShootMotor.setPower(0);
       
        IntakeServo3.setPosition(0.3);//was 0.4
        AprilTagLibrary tagLibrary = new AprilTagLibrary.Builder()
                .addTag(24, "redgoal", 0.1651, DistanceUnit.METER)
                .build();

        aprilTag = new AprilTagProcessor.Builder()
                .setTagLibrary(tagLibrary)
                .setLensIntrinsics(fx, fy, cx, cy)
                .build();

        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam"),
                aprilTag
        );

        runtime.reset();
        lastTime = runtime.seconds();
    }

    /* ---------------- LOOP ---------------- */
    @Override
    public void loop() {

        currentVoltage = hardwareMap.voltageSensor.iterator().next().getVoltage();

        if (currentVoltage > 13.0)      kF = 0.80 / 3750;
        else if (currentVoltage > 12.2) kF = 0.80 / 3500;
        else if (currentVoltage > 11) kF = 0.80 / 3200;
        else                            kF = 0.80 / 3000;


        /* -------- APRILTAG -------- */
        tagVisible = false;
        tagDistanceInches = -1;
        tagYaw = 0;

        for (AprilTagDetection d : aprilTag.getDetections()) {
            if (d.id == TARGET_TAG_ID) {
                tagVisible = true;
                tagDistanceInches = d.ftcPose.range-1.25;
                break;
            }
        }

        if (camtargeton && tagVisible && tagDistanceInches > 0) {
            updateFlywheelRPMFromAprilTag();
        }

        /* -------- DRIVE -------- */
        double y  = -gamepad1.left_stick_y;
        double x  =  gamepad1.left_stick_x;
        double rx =  (gamepad1.right_stick_x)*0.6;



        double den = Math.max(1.0, Math.abs(y) + Math.abs(x) + Math.abs(rx));

        FrontLeftWheel.setPower((y + x + rx) / den);
        FrontRightWheel.setPower((y - x - rx) / den);
        BackLeftWheel.setPower((y - x + rx) / den);
        BackRightWheel.setPower((y + x - rx) / den);

        /* -------- TOGGLES -------- */
        
        // ---- LIFT TOGGLE (X button) ----
        if (!gamepad2.x) liftToggleReady = true;

        if (gamepad2.x && liftToggleReady) {
            
            liftUp = !liftUp;
            liftToggleReady = false;
        }
        
        // ---- LIFT TARGET ----
        if (liftUp) {
            LiftMotor.setTargetPosition(-1600); // UP
        } else {
            LiftMotor.setTargetPosition(0);    // DOWN
        }
        
        // ---- LIFT POWER ----
        if (LiftMotor.isBusy()) {
            LiftMotor.setPower(1.0);
        } else {
            LiftMotor.setPower(0.0);
        }

        if (!gamepad2.a) intakeToggleReady = true;
        if (gamepad2.a && intakeToggleReady) {
            intakeOn = !intakeOn;
            intakeToggleReady = false;
        }
        IntakeMotor.setPower(intakeOn ? -1 : 0);

        if (!gamepad2.b) flyToggleReady = true;
        if (gamepad2.b && flyToggleReady) {
            flywheelOn = !flywheelOn;
            flyToggleReady = false;
        }
                // 2000 RPM
        if (!gamepad2.dpad_down) rpm2000Ready = true;
        if (gamepad2.dpad_down && rpm2000Ready) {
            flyspeed2000 = true;
            flyspeed2400 = flyspeed2800 = flyspeed2200 = false;
            camtargeton = false;
            targetFlyRPM = 2000;
            rpm2000Ready = false;
        }
        
        // 2200 RPM
        if (!gamepad2.dpad_left) rpm2200Ready = true;
        if (gamepad2.dpad_left && rpm2200Ready) {
            flyspeed2200 = true;
            flyspeed2000 = flyspeed2400 = flyspeed2800 = false;
            camtargeton = false;
            targetFlyRPM = 2200;
            rpm2400Ready = false;
        }
        
        // 2400 RPM
        if (!gamepad2.dpad_up) rpm2400Ready = true;
        if (gamepad2.dpad_up && rpm2400Ready) {
            flyspeed2400 = true;
            flyspeed2000 = flyspeed2200 = flyspeed2400 = false;
            camtargeton = false;
            targetFlyRPM = 2800;
            rpm2400Ready = false;
        }
        
        // 2800 RPM
        if (!gamepad2.dpad_right) rpm2800Ready = true;
        if (gamepad2.dpad_right && rpm2800Ready) {
            flyspeed2800 = true;
            flyspeed2000 = flyspeed2400 = flyspeed2200 = false;
            camtargeton = false;
            targetFlyRPM = 3000;
            rpm2800Ready = false;
        }
        if (!gamepad2.left_bumper) camToggleReady = true;
        if (gamepad2.left_bumper && camToggleReady) {
            camtargeton = true;
        
            flyspeed2000 = false;
            flyspeed2200 = false;
            flyspeed2400 = false;
            flyspeed2800 = false;
        
            camToggleReady = false;
        }

              
            double trigger = gamepad2.right_trigger;
        
        // Detect trigger release
        if (trigger < 0.05) {
            triggerReleased = true;
        }
        
        // Start ram sequence
        if (trigger > 0.1 && triggerReleased ) {
            shootticks-=-179;
            triggerReleased = false;
            
            
            
            IntakeServo3.setPosition(1);//was1
            ShootMotor.setTargetPosition(shootticks); // Extend first
            
            servoTimer.reset();
        }
        
        // State 3: delay, then retract Servo 3
        if (servoTimer.seconds() >= 0.5) {//takes .75 usually to turn 180
            ShootMotor.setPower(1.0); 
            
            
            if(servoTimer.seconds() >=1){
                IntakeServo3.setPosition(0.3);
                ShootMotor.setPower(0.0);
            }
            
        }

        
    
    

    
    
        /* -------- FLYWHEEL PID -------- */
        double now = runtime.seconds();
        double dt = now - lastTime;

        if (dt >= 0.02) {
            rpmRaw = (FlyWheel.getCurrentPosition() - lastFly) *
                    (60.0 / (TICKS_PER_REV * dt));

            filteredRPM = 0.8 * filteredRPM + 0.2 * rpmRaw;

            lastFly = FlyWheel.getCurrentPosition();
            lastTime = now;

            if (flywheelOn) {
                double error = targetFlyRPM - filteredRPM;
                flyIntegral = Math.max(-flyIntegralMax,
                        Math.min(flyIntegralMax, flyIntegral + error * dt));

                double derivative = (error - flyLastError) / dt;

                double output =
                        kF * targetFlyRPM +
                        kP * error +
                        kI * flyIntegral +
                        kD * derivative;

                FlyWheel.setPower(Math.max(0, Math.min(1, output)));
                flyLastError = error;
            } else {
                FlyWheel.setPower(0);
                flyIntegral = 0;
                flyLastError = 0;
            }
        }
        
        telemetry.addData("Voltage", currentVoltage);
        telemetry.addData("Yaw", tagYaw);
        telemetry.addData("Tag Visible", tagVisible);
        telemetry.addData("Distance(Inches)",tagDistanceInches );
        telemetry.addData("Target RPM", targetFlyRPM);
        telemetry.addData("RPM", filteredRPM);
        telemetry.addData("Cam RPM Mode", camtargeton);
        telemetry.addData("RPM Mode",
                flyspeed2000 ? "2000" :
                flyspeed2200 ? "2200" :
                flyspeed2400 ? "2400" :
                flyspeed2800 ? "3000" : "Camera");

        telemetry.update();
    }

    @Override
    public void stop() {
        if (visionPortal != null) visionPortal.close();
    }
}




