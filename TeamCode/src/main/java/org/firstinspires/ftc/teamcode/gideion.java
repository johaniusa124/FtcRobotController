package org.firstinspires.ftc.teamcode;

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
import java.util.List;

@TeleOp

public class FINAL_TELEOP_YAY_2 extends LinearOpMode{

    DcMotorEx frontLeftMotor;
    DcMotorEx frontRightMotor;
    DcMotorEx backLeftMotor;
    DcMotorEx backRightMotor;

    DcMotorEx leftSpinnerMotor;
    DcMotorEx rightSpinnerMotor;

    DcMotorEx feederMotor;

    DcMotorEx lifterMotor;

    Servo handServo;
    Servo handServo2;

    CRServo bootKicker;

    Servo pusherServo;

    TouchSensor armLimit;

    Limelight3A limelight;

    private VisionPortal visionportal;
    private VisionPortal visionportal2;
    private AprilTagProcessor tagProcessor;

    double speedMult = 3000;
    double smoothingValue = 0.075;
    double deadSmoothingSize = 0.005;
    double diffCoefficient = 0.001;

    boolean shouldLaunch = false;
    boolean toggled = false;
    boolean toggled2 = false;
    boolean toggled3 = false;
    boolean goingUp = false;

    double frontLeftSpeed = 0;
    double frontRightSpeed = 0 ;
    double backLeftSpeed = 0;
    double backRightSpeed = 0;

    double launchSpeedHigh = 1050;
    double launchSpeedLow = 1200;

    double invertMovement = 1;

    private double getNewVelocity(double current, double target) {
        return target;
        /*
        double diff = target - current;
        if ((Math.abs(target) <= deadSmoothingSize)&&(Math.abs(diff) <= deadSmoothingSize)) {
        // telemetry.addData("it's dead", 304202);
        return 0;
        }
        double delta = Math.copySign(smoothingValue, diff);

        // telemetry.addData("diff", diff);

        return current + delta;
        */
    }
    private void initAprilTags() {
        AprilTagProcessor.Builder myAprilTagProcessorBuilder;
        AprilTagProcessor myAprilTagProcessor;

        myAprilTagProcessorBuilder = new AprilTagProcessor.Builder();

        myAprilTagProcessorBuilder.setLensIntrinsics(711.636, 711.636, 320.986, 229.45);

        tagProcessor = myAprilTagProcessorBuilder.build();

        // tagProcessor = AprilTagProcessor.easyCreateWithDefaults();

        // VisionPortal visionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 2"), tagProcessor);

        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch START to start OpMode");
        telemetry.update();
    }
    private void updateAprilTags() {
        List <AprilTagDetection> currentDetections = tagProcessor.getDetections();
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
        // telemetry.update();
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

        frontLeftMotor.setVelocity(frontLeftSpeed*speedMult*invertMovement);
        frontRightMotor.setVelocity(frontRightSpeed*speedMult*invertMovement);
        backLeftMotor.setVelocity(backLeftSpeed*speedMult*invertMovement);
        backRightMotor.setVelocity(backRightSpeed*speedMult*invertMovement);
    }

    @Override
    public void runOpMode() throws InterruptedException {

        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeftMotor = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRightMotor = hardwareMap.get(DcMotorEx.class, "backRight");

        leftSpinnerMotor = hardwareMap.get(DcMotorEx.class, "leftSpinner");
        rightSpinnerMotor = hardwareMap.get(DcMotorEx.class, "rightSpinner");

        feederMotor = hardwareMap.get(DcMotorEx.class, "intake");

        lifterMotor = hardwareMap.get(DcMotorEx.class, "tpose");

        handServo = hardwareMap.get(Servo.class, "hand");
        handServo2 = hardwareMap.get(Servo.class, "hand2");

        bootKicker = hardwareMap.get(CRServo.class, "bootKicker");

        armLimit = hardwareMap.get(TouchSensor.class, "armLimit");

        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftSpinnerMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightSpinnerMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftSpinnerMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        //bootKicker.setDirection(com.qualcomm.robotcore.hardware.Servo);
        //bootKicker2.setDirection(com.qualcomm.robotcore.hardware.Servo);


        feederMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        lifterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(0);

        limelight.start();


        initAprilTags();

        PredominantColorProcessor colorSensor = new PredominantColorProcessor.Builder()
        .setRoi(ImageRegion.asUnityCenterCoordinates(-1, 1, 1, -1))
        .setSwatches(
        PredominantColorProcessor.Swatch.ARTIFACT_GREEN,
        PredominantColorProcessor.Swatch.ARTIFACT_PURPLE,
        PredominantColorProcessor.Swatch.RED,
        PredominantColorProcessor.Swatch.BLUE,
        PredominantColorProcessor.Swatch.YELLOW,
        PredominantColorProcessor.Swatch.BLACK,
        PredominantColorProcessor.Swatch.WHITE)
        .build();

        VisionPortal visionPortalButColor = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 2"), colorSensor);


        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            boolean redGoalTag = false;
            boolean blueGoalTag = false;
            LLResult result = limelight.getLatestResult();
            if (result != null) {
                // telemetry.addData("limelight result", result.toString());
                List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
                for (LLResultTypes.FiducialResult fr : fiducialResults) {
                    if (result.getFiducialResults() != null) {
                        if (fr.getFiducialId() == 24) {
                            redGoalTag = true;
                        }
                        if (fr.getFiducialId() == 20) {
                            blueGoalTag = true;
                        }
                    }
                }
            }

            telemetry.addData("red", redGoalTag);
            telemetry.addData("blue", blueGoalTag);

            // updateAprilTags();

            List <AprilTagDetection> currentDetections = tagProcessor.getDetections();


            AprilTagDetection redGoal = null;
            AprilTagDetection blueGoal = null;
            for (AprilTagDetection detection : currentDetections) {
                if (detection.id == 24) {
                    redGoal = detection;
                }
                if (detection.id == 20) {
                    blueGoal = detection;
                }
            }

            double y = gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = invertMovement*gamepad1.right_stick_x;

            double paddleSpeed = gamepad2.right_stick_x;

            y *= 1-gamepad1.left_trigger;
            x *= 1-gamepad1.left_trigger;
            rx *= 1-gamepad1.left_trigger;

            boolean yesLaunch = gamepad2.a;

            if (gamepad1.y) {
                if (!toggled3) {
                    invertMovement *= -1;
                    toggled3 = true;
                }
            } else {
                toggled3 = false;
            }

            if ((toggled == false)&&(yesLaunch)) {
                toggled = true;
                shouldLaunch = !shouldLaunch;
            } else if (!yesLaunch) {
                toggled = false;
            }


            if (gamepad2.dpad_up) {
                launchSpeedHigh += 1;
                launchSpeedLow +=1;
            } else if (gamepad2.dpad_down) {
                launchSpeedHigh -= 1;
                launchSpeedLow -= 1;
            }

            // alignment
            if (gamepad2.x && redGoalTag) {
                telemetry.addData("sees red", "yeah");
                double bearing = result.getTx();
                double yaw = 0;
                // double bearing = redGoal.ftcPose.bearing;
                // double yaw = redGoal.ftcPose.yaw;
                bearing = bearing+yaw/6;
                double wheelBaseRadius = 10.25*2.54/2;
                double wheelRadius = 3.125*2.54/2;
                double distance = wheelBaseRadius/wheelRadius*bearing;
                double circumference = wheelRadius*2*Math.PI;
                double rotations = distance/circumference;
                double ticks = rotations*28;

                frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                frontLeftMotor.setTargetPosition((int)(-ticks));
                frontRightMotor.setTargetPosition((int)(-ticks));
                backLeftMotor.setTargetPosition((int)(ticks));
                backRightMotor.setTargetPosition((int)(ticks));
                telemetry.addData("ticks", ticks);

                frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                frontLeftMotor.setVelocity(1000);
                frontRightMotor.setVelocity(1000);
                backLeftMotor.setVelocity(1000);
                backRightMotor.setVelocity(1000);

                // if (Math.abs(redGoal.ftcPose.yaw) >= 1) {
                // telemetry.addData("needs mov", "mhm!");
                // double movementAmount = -Math.copySign(.25, redGoal.ftcPose.yaw);
                // telemetry.addData("horizontal", movementAmount);
                // moveInDirection(movementAmount,0,0);
                // telemetry.addData("holy crap", "it working");
                // }

            } else {
                frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                moveInDirection(x, y, rx);
            }

            if (shouldLaunch && redGoalTag) {
                double range = result.getTa();

                double term1 = -0.014098*Math.pow(range,2);
                double term2 = 7.61821*range;
                double term3 = 827.55741;

                double speed = term1+term2+term3;
                telemetry.addData("tag a value idk", range);
                telemetry.addData("term1", term1);
                telemetry.addData("term2", term2);
                telemetry.addData("term3", term3);


                /* this stuff is not really working, the rest of this area is the regression-based aiming
                double wheelRadius = 3.125*2.54/2;
                double wheelCircumference = wheelRadius*Math.PI*2;

                double range = redGoal.ftcPose.range*.0254;
                double g = 9.8;
                double thetaDegrees = 67;
                double h = 110;

                double numerator = g*range*range/Math.cos(thetaDegrees);
                double denominator = 2*(range*Math.tan(thetaDegrees)-h);

                double angularVelocity = Math.sqrt(numerator/denominator);

                double speed = angularVelocity/wheelCircumference*28*1225*2;

                telemetry.addData("auto-adjusting speed to",speed);
                */

                // UNCOMMENT THIS ONCE YOU'RE DONE WITH THE REGRESSION PLEASE
                // launchSpeedHigh = speed;
                // UNCOMMENT THIS ONCE YOU'RE DONE WITH THE REGRESSION PLEASE
            }


            feederMotor.setVelocity((int)(-4000*gamepad2.left_stick_y));
            if (shouldLaunch) {
                leftSpinnerMotor.setVelocity((int)(launchSpeedHigh));
                rightSpinnerMotor.setVelocity((int)(launchSpeedHigh));
            } if (!shouldLaunch) {
                leftSpinnerMotor.setVelocity((int)(launchSpeedLow)*0);
                rightSpinnerMotor.setVelocity((int)(launchSpeedLow)*0);
            }
            if (gamepad2.dpad_up){
                bootKicker.setPower(1);
            }
            if (gamepad2.dpad_down){
                bootKicker.setPower(0);

            }
            if (gamepad2.y && toggled2 == false) {
                goingUp = !goingUp;
                toggled2 = true;
            } else if (!gamepad2.y) {
                toggled2 = false;
            }
            if (goingUp && armLimit.isPressed()) {
                goingUp = false;

            }


            double downPos = 0.445;
            double upPos = 0.75;

            if (goingUp) {
                handServo.setPosition(upPos);
                handServo2.setPosition(1-upPos);
                telemetry.addData("servo 1 position", handServo.getPosition());
                telemetry.addData("servo 2 position", handServo2.getPosition());
            } else {
                handServo.setPosition(downPos);
                handServo2.setPosition(1-downPos);
                telemetry.addData("servo 1 position", handServo.getPosition());
                telemetry.addData("servo 2 position", handServo2.getPosition());
            }

            lifterMotor.setVelocity((int)((gamepad2.left_trigger-gamepad2.right_trigger)*1000000));


            telemetry.addData("color(?)", colorSensor.getAnalysis().closestSwatch);

            telemetry.addData("speed", launchSpeedHigh);

            telemetry.update();

            }

        }

}