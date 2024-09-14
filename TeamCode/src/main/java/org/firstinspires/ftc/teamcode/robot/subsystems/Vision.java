package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class Vision implements Subsystem {
    private final AprilTagProcessor aprilTagProcessor;
    private final VisionPortal portal;
    private final Vector2d cameraOffset = new Vector2d(5.3, 0);

    public Vision(HardwareMap hardwareMap) {

        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .build();

        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTagProcessor)
                .setAutoStopLiveView(true)
                .build();
    }

    public List<AprilTagDetection> getDetections() {
        if (portal.getCameraState() != VisionPortal.CameraState.STREAMING)
            return new ArrayList<>();

        return aprilTagProcessor.getDetections();
    }


    public void shutdown() {
        if (portal.getCameraState() == VisionPortal.CameraState.CAMERA_DEVICE_CLOSED)
            return;

        portal.close();
    }
    @Override
    public void update() {

    }
}

