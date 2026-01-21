package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.mechanisims.AprilTagWebcam;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

public class AprilTagEG extends OpMode {

    AprilTagWebcam aprilTagWebcam = new AprilTagWebcam();

    @Override
    public void init() {
        aprilTagWebcam.init(hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        aprilTagWebcam.update();
        AprilTagDetection id20 = aprilTagWebcam.getTagByID(20);
        aprilTagWebcam.displayAprilTelem(id20);
    }
}
