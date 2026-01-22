package org.firstinspires.ftc.teamcode.mechanisims;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class DriveBase {
    public DcMotorEx frontLeftMotor;
    public DcMotorEx backLeftMotor;
    public DcMotorEx frontRightMotor;
    public DcMotorEx backRightMotor;


    public void init(HardwareMap hwMap, boolean reverseLeftMotors) {
        frontLeftMotor = hwMap.get(DcMotorEx.class, "frontLeftMotor");
        backLeftMotor = hwMap.get(DcMotorEx.class, "backLeftMotor");
        frontRightMotor = hwMap.get(DcMotorEx.class,"frontRightMotor");
        backRightMotor = hwMap.get(DcMotorEx.class,"backRightMotor");

        if (reverseLeftMotors) {
            frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        // Common: reverse right motors if your robot config needs it.
        // frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        // backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        /*// Initialize IMU
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = AngleUnit.DEGREES;
        parameters.accelUnit = com.qualcomm.robotcore.external.navigation.AccelerationUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        imu = HardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);*/
    }

    public double pitchAim(int rpm, double dist, double height) {
        double r = .2;
        double V = rpm * r;
        double g = 9.8;
        double T = Math.acos((-9.8 * dist) / (V * (((9.8 * height) / (2 * V)) + Math.sqrt(((96.04 * r*r) / (4 * V * V)) - (19.6 * height)))));
        return T;
    }


}
