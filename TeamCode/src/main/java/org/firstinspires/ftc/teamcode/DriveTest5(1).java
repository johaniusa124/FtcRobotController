package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.VisionPortal;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import java.util.List;

/* ORIG IMPORTS
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime; */


/* GIDEONS IMPORTS
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.CRServo;
import org.firstinspires.ftc.vision.opencv.PredominantColorProcessor;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import com.qualcomm.robotcore.hardware.TouchSensor;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.VisionPortal;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import java.util.List; */

@TeleOp

public class DriveTest5 extends LinearOpMode {

    DcMotorEx frontLeftMotor;
    DcMotorEx frontRightMotor;
    DcMotorEx backLeftMotor;
    DcMotorEx backRightMotor;

    DcMotorEx flywheel1;
    DcMotorEx flywheel2;

    DcMotorEx intake;

    Servo flyAdjust;

    Servo recycle;

    CRServo move1;
    CRServo move2;
    CRServo move3;
    CRServo moveMid;

    //ADD USB CAMREA


    //VARIABLES
    double frontLeftSpeed = 0;
    double frontRightSpeed = 0 ;
    double backLeftSpeed = 0;
    double backRightSpeed = 0;

    boolean isIntake = false; 
    boolean isFlywheel = false;
    boolean isRecycling = false;

    double flyAdjustAngle = 0;


    //PID tuning

    double highVelocity = 1500;
    double lowVelocity = 900;

    double currTargetVelocity = highVelocity;

    double P = 0;
    double F = 0;

    double[] stepSizes = {10.0, 1.0, .1, .01, .001, .0001};

    int stepIndex = 1;

    private void initAprilTags() {
        /*AprilTagProcessor.Builder myAprilTagProcessorBuilder;
        AprilTagProcessor myAprilTagProcessor;

        myAprilTagProcessorBuilder = new AprilTagProcessor.Builder();

        myAprilTagProcessorBuilder.setLensIntrinsics(711.636, 711.636, 320.986, 229.45);

        tagProcessor = myAprilTagProcessorBuilder.build();*/

        // tagProcessor = AprilTagProcessor.easyCreateWithDefaults();

        // VisionPortal visionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 2"), tagProcessor);

        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch START to start OpMode");
        telemetry.update();
    }

    private void updateAprilTags() {
        /*List <AprilTagDetection> currentDetections = tagProcessor.getDetections();
        telemetry.addData("number of tags detected", currentDetections.size());

        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");
        // telemetry.update(); */
    }
    
    private double getNewVelocity(double current, double target) {
        return target;
        
    }
    private void moveInDirection(double x, double y, double rx) {
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

        double frontLeftTarget = (x - y - rx) / denominator;
        double backLeftTarget = (x + y + rx) / denominator;
        double frontRightTarget = (x + y - rx) / denominator;
        double backRightTarget = (x - y + rx) / denominator;

        frontLeftSpeed = getNewVelocity(frontLeftSpeed, frontLeftTarget);
        frontRightSpeed = getNewVelocity(frontRightSpeed, frontRightTarget);
        backLeftSpeed = getNewVelocity(backLeftSpeed, backLeftTarget);
        backRightSpeed = getNewVelocity(backRightSpeed, backRightTarget);

        frontLeftMotor.setVelocity(frontLeftSpeed);
        frontRightMotor.setVelocity(frontRightSpeed);
        backLeftMotor.setVelocity(backLeftSpeed);
        backRightMotor.setVelocity(backRightSpeed);
    }



    @Override
    public void runOpMode(){
        
        // INITIATION
        // DC Motors
        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "frontLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "frontRightMotor");
        backLeftMotor = hardwareMap.get(DcMotorEx.class, "backLeftMotor");
        backRightMotor = hardwareMap.get(DcMotorEx.class, "backRightMotor");

        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        flywheel1 = hardwareMap.get(DcMotorEx.class, "flywheel1");
        flywheel2 = hardwareMap.get(DcMotorEx.class, "flywheel2");

        flywheel2.setDirection(DcMotorSimple.Direction.REVERSE);

        flywheel1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        

        intake = hardwareMap.get(DcMotorEx.class, "intake");

        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Servos
        flyAdjust = hardwareMap.get(Servo.class, "flyAdjust");
        recycle = hardwareMap.get(Servo.class, "recycle");

        move1 = hardwareMap.get(CRServo.class, "move1");
        move2 = hardwareMap.get(CRServo.class, "move2");
        move3 = hardwareMap.get(CRServo.class, "move3");
        moveMid = hardwareMap.get(CRServo.class, "moveMid");

        //Camrea stuff
        initAprilTags();
        

        //PID Stuff
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);

        flywheel1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        flywheel2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        telemetry.addLine("INIT Complete");

        waitForStart();

        if (isStopRequested()) return;


        while (opModeIsActive()) {


            //PID Stuff

            if (gamepad1.yWasPressed()){
                if (currTargetVelocity == highVelocity){
                    currTargetVelocity=lowVelocity;
                } else { 
                    currTargetVelocity=highVelocity;
                }
            }

            if (gamepad1.bWasPressed()){
                stepIndex = (stepIndex++)% stepSizes.length;
            }

            if (gamepad1.dpadLeftWasPressed()){
                F -= stepSizes[stepIndex];
            }

            if (gamepad1.dpadRightWasPressed()){
                F += stepSizes[stepIndex];
            }

            if (gamepad1.dpadUpWasPressed()){
                P += stepSizes[stepIndex];
            }

            if (gamepad1.dpadDownWasPressed()){
                P -= stepSizes[stepIndex];
            }

            PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);

            flywheel1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
            flywheel2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

            flywheel1.setVelocity(currTargetVelocity);
            flywheel2.setVelocity(currTargetVelocity);

            double currVelocity1 = flywheel1.getVelocity();
            double currVelocity2 = flywheel2.getVelocity();

            double error1 = currTargetVelocity - currVelocity1;
            double error2 = currTargetVelocity - currVelocity2;

            telemetry.addData("Target Velocity", currTargetVelocity);
            telemetry.addData("Curr Velo 1", "%.2f", currVelocity1);
            telemetry.addData("err 1", ".2f", error1);

            telemetry.addData("Curr Velo 2", "%.2f", currVelocity2);
            telemetry.addData("err 2", ".2f", error2);

            telemetry.addData("tuning P", "%.4f (D-Pad U/D)", P);
            telemetry.addData("tuning F", "%.4f (D-Pad L/R)", F);
            telemetry.addData("Step Size", "%.4f (B)", stepSizes[stepIndex]);
            //List <AprilTagDetection> currentDetections = tagProcessor.getDetections();

            //Driving Stuff/gamepad1
            double y = gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            moveInDirection(x, y, rx);

            //Gamepad2
            intake.setVelocity(gamepad2.left_stick_y);

            if (gamepad2.a){
                if (isFlywheel){
                    flywheel1.setVelocity(0);
                    flywheel2.setVelocity(0);
                    isFlywheel = false;
                } else {
                    flywheel1.setVelocity(1000);
                    flywheel2.setVelocity(1000);
                    isFlywheel = true;
                }
            }

            if (gamepad2.b){
                if (isRecycling){
                    recycle.setPosition(.5);
                    isRecycling=false;
                } else {
                    recycle.setPosition(1);
                    isRecycling=true;
                }
            }


            if (gamepad2.left_trigger>0){
                //flyAdjust.setPower(gamepad2.left_trigger);
            } else if (gamepad2.right_trigger>0){
                //flyAdjust.setPower(gamepad2.right_trigger*-1);
            } else {
                //flyAdjust.setPower(0);
            }
            
            moveMid.setPower(gamepad2.right_stick_y);
            

            /*IF flyADjust is not a CRServo
            if (gamepad2.left_trigger>0){
                flyAdjustAngle+=.1;
                flyAdjust.setPosition(flyAdjustAngle);
            } else if (gamepad2.right_trigger>0){
                flyAdjustAngle-=.1;
                flyAdjust.setPosition(flyAdjustAngle);
            } */

            //Telemetry?
            telemetry.update();
        }

    }

}
