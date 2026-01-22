package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.mechanisims.AprilTagWebcam;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;


@TeleOp
public class Drive6 extends OpMode {

    DcMotorEx frontLeftMotor;
    DcMotorEx frontRightMotor;
    DcMotorEx backLeftMotor;
    DcMotorEx backRightMotor;

    DcMotorEx flywheel1;
    DcMotorEx flywheel2;

    DcMotorEx intake;

    CRServo flyAdjust;

    Servo recycle;

    CRServo move1;
    CRServo move2;
    CRServo move3;
    CRServo moveMid;

    //ADD USB CAMREA
    AprilTagWebcam aprilTagWebcam = new AprilTagWebcam();

    //VARIABLES
    double frontLeftSpeed = 0;
    double frontRightSpeed = 0 ;
    double backLeftSpeed = 0;
    double backRightSpeed = 0;

    double intakeSpeed = 0;

    boolean isIntake = false;
    boolean isFlywheel = false;
    boolean isRecycling = false;
    boolean isSorting = false;
    boolean isAutoPitchAiming = false;
    boolean isAdjusting = false;

    boolean RedAlliance = Boolean.parseBoolean(null);
    boolean redGoalTag = false;
    boolean blueGoalTag = false;

    double flyAdjustAngle = 0;
    double yaw = -1000;
    double bearing = -1000;
    double dist = -1000;
    double height = 50;

    double startTime;
    double finishTime;
    double servoRuntime;
    double servoSpeed = 428.571; //Degrees per sec
    double targetAngle;

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

    public double pitchAim(int rpm, double dist, double height) {
        double r = .2;
        double V = rpm * r;
        double g = 9.8;
        double T = Math.acos((-9.8 * dist) / (V * (((9.8 * height) / (2 * V)) + Math.sqrt(((96.04 * r*r) / (4 * V * V)) - (19.6 * height)))));
        return T;
    }



    @Override
    public void init() {

        // DC Motors
        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeftMotor = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRightMotor = hardwareMap.get(DcMotorEx.class, "backRight");

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
        flyAdjust = hardwareMap.get(CRServo.class, "flyAdjust");
        recycle = hardwareMap.get(Servo.class, "recycle");

        move1 = hardwareMap.get(CRServo.class, "move1");
        move2 = hardwareMap.get(CRServo.class, "move2");
        move3 = hardwareMap.get(CRServo.class, "move3");
        moveMid = hardwareMap.get(CRServo.class, "moveMid");

        //Camrea stuff
        aprilTagWebcam.init(hardwareMap, telemetry);
        //initAprilTags();

        //aiming Servo
        //zero
        flyAdjust.setPower(.5);
        flyAdjustAngle = 10;

    }

    @Override
    public void loop() {

        //camera
        aprilTagWebcam.update();
        AprilTagDetection id20 = aprilTagWebcam.getTagByID(20);
        AprilTagDetection id24 = aprilTagWebcam.getTagByID(24);
        if (RedAlliance){
            if (id24 != null){
                bearing = aprilTagWebcam.returnBearing(id24);
                yaw = aprilTagWebcam.returnYaw(id24);
            }
        } else if (!RedAlliance){
            if (id20 != null){
                bearing = aprilTagWebcam.returnBearing(id20);
                yaw = aprilTagWebcam.returnYaw(id20);
            }
        }


        //movement
        double y = gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        moveInDirection(x, y, rx);

        if (gamepad2.y && bearing!=-1000) {
            telemetry.addData("sees red", "yeah");
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

        //intake
        if (gamepad1.bWasPressed()){
            if (isIntake){
                isIntake = false;
                intakeSpeed=0;
            } else {
                isIntake = true;
                intakeSpeed = 1500;
            }
        }

        if (gamepad2.right_stick_y!=0){
            intakeSpeed = gamepad2.right_stick_y*1500;
        }

        intake.setVelocity(intakeSpeed);

        //flywheel
        if (gamepad2.aWasPressed()){
            if (isFlywheel){
                flywheel1.setVelocity(0);
                flywheel2.setVelocity(0);
                isFlywheel = false;
            } else {
                flywheel1.setVelocity(1500);
                flywheel2.setVelocity(1500);
                isFlywheel = true;
            }
        }

        //recycling
        if (gamepad2.bWasPressed()){
            if (isRecycling){
                recycle.setPosition(.5);
                isRecycling=false;
            } else {
                recycle.setPosition(1);
                isRecycling=true;
            }
        }

        //sorting
        if (gamepad2.xWasPressed()) {
            if (isSorting){
                isSorting = false;
                moveMid.setPower(0);
                move1.setPower(0);
                move2.setPower(0);
                move3.setPower(0);
            } else {
                isSorting = true;
                moveMid.setPower(1);
                move1.setPower(1);
                move2.setPower(1);
                move3.setPower(1);
            }
        }

        //aiming

        if (isAutoPitchAiming) {
            if (!isAdjusting) {
                if (RedAlliance) {
                    if (id24 != null) {
                        targetAngle = pitchAim((int) flywheel1.getVelocity(), dist / 100, height / 100);

                        servoRuntime = Math.abs(targetAngle - flyAdjustAngle) / servoSpeed;

                        startTime = getRuntime();
                        finishTime = startTime + servoRuntime;

                        if (targetAngle - flyAdjustAngle > 0) {
                            flyAdjust.setPower(1);
                        } else if (targetAngle - flyAdjustAngle < 0) {
                            flyAdjust.setPower(-1);
                        }

                        isAdjusting = true;

                    }
                } else if (!RedAlliance) {
                    if (id20 != null) {
                        targetAngle = pitchAim((int) flywheel1.getVelocity(), dist / 100, height / 100);

                        servoRuntime = Math.abs(targetAngle - flyAdjustAngle) / servoSpeed;

                        startTime = getRuntime();
                        finishTime = startTime + servoRuntime;

                        if (targetAngle - flyAdjustAngle > 0) {
                            flyAdjust.setPower(1);
                        } else if (targetAngle - flyAdjustAngle < 0) {
                            flyAdjust.setPower(-1);
                        }

                        isAdjusting = true;
                    }
                }
            }
        }

        if (getRuntime()>=finishTime){
            isAdjusting = false;
            flyAdjust.setPower(0);
        }



        if (!isAutoPitchAiming) {
            if (gamepad2.left_trigger > 0) {
                flyAdjust.setPower(gamepad2.left_trigger);
            } else if (gamepad2.right_trigger > 0) {
                flyAdjust.setPower(gamepad2.right_trigger * -1);
            } else {
                flyAdjust.setPower(0);
            }
        }

        if (gamepad2.leftBumperWasPressed()) {
            if (isAutoPitchAiming) {
                isAutoPitchAiming = false;
            } else {
                isAutoPitchAiming = true;
            }
        }



    }
}
