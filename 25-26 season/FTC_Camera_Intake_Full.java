//  package org.firstinspires.ftc.teamcode;

// import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
// import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
// import com.qualcomm.robotcore.hardware.DcMotor;
// import com.qualcomm.robotcore.hardware.CRServo;

// import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
// import org.firstinspires.ftc.vision.VisionPortal;
// import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
// import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

// import java.util.List;

// @Autonomous(name = "FTC_Camera_Intake_Full")
// public class FTC_Camera_Intake_Full extends LinearOpMode {

//     DcMotor FrontLeftWheel, FrontRightWheel, BackRightWheel, BackLeftWheel;
//     DcMotor IntakeMotor;
//     private CRServo IntakeServo1;

//     private VisionPortal visionPortal;
//     private AprilTagProcessor aprilTag;

//     enum AutoState {SCAN, MOVE_TO_TAG, PARK}
//     AutoState currentState = AutoState.SCAN;

//     private void drive(double power, long timeMs) {
//         FrontLeftWheel.setPower(power);
//         FrontRightWheel.setPower(power);
//         BackLeftWheel.setPower(power);
//         BackRightWheel.setPower(power);
//         sleep(timeMs);
//         stopDrive();
//     }

//     private void strafe(double power, long timeMs) {
//         FrontLeftWheel.setPower(power);
//         BackLeftWheel.setPower(-power);
//         FrontRightWheel.setPower(-power);
//         BackRightWheel.setPower(power);
//         sleep(timeMs);
//         stopDrive();
//     }

//     private void stopDrive() {
//         FrontLeftWheel.setPower(0);
//         FrontRightWheel.setPower(0);
//         BackLeftWheel.setPower(0);
//         BackRightWheel.setPower(0);
//     }

//     @Override
//     public void runOpMode() {

       
//         FrontLeftWheel  = hardwareMap.dcMotor.get("FrontLeft");
//         FrontRightWheel = hardwareMap.dcMotor.get("FrontRight");
//         BackRightWheel  = hardwareMap.dcMotor.get("BackRight");
//         BackLeftWheel   = hardwareMap.dcMotor.get("BackLeft");
//         IntakeMotor     = hardwareMap.dcMotor.get("intakemotor");

//         FrontRightWheel.setDirection(DcMotor.Direction.REVERSE);
//         BackRightWheel.setDirection(DcMotor.Direction.REVERSE);

//         IntakeServo1 = hardwareMap.get(CRServo.class, "intakeservo");

      
//         aprilTag = AprilTagProcessor.easyCreateWithDefaults();
//         WebcamName webcam = hardwareMap.get(WebcamName.class, "Webcam");

//         visionPortal = new VisionPortal.Builder()
//                 .setCamera(webcam)
//                 .addProcessor(aprilTag)
//                 .build();

       
//         String[] ballPattern = new String[3];
//         int currentBallIndex = 0;

//         waitForStart();

//         while (opModeIsActive()) {

//             List<AprilTagDetection> detections = aprilTag.getDetections();

//             switch (currentState) {

                
//                 case SCAN:
//                     if (detections.size() > 0) {
//                         currentBallIndex = 0;
//                         currentState = AutoState.MOVE_TO_TAG;
//                     }
//                     break;

                
//                 case MOVE_TO_TAG:
//                     if (detections.size() == 0) {
//                         currentState = AutoState.SCAN;
//                         break;
//                     }

//                     AprilTagDetection tag = detections.get(0);

                   
//                     if (tag.ftcPose.x > 2) {
//                         strafe(0.3, 200);
//                     } else if (tag.ftcPose.x < -2) {
//                         strafe(-0.3, 200);
//                     } else if (tag.ftcPose.y > 8) {
//                         drive(0.4, 300);
//                     } else {
                        
//                         stopDrive();

                        
//                         IntakeMotor.setPower(1);
//                         sleep(1500);
//                         IntakeMotor.setPower(0);

//                         currentBallIndex++;
//                         if (currentBallIndex >= ballPattern.length) {
//                             currentState = AutoState.PARK;
//                         } else {
//                             currentState = AutoState.MOVE_TO_TAG;
//                         }
//                     }
//                     break;

            
//                 case PARK:
//                     stopDrive();
//             `   `        break;
//             }
//         }
//     }
// }

