package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.acmerobotics.roadrunner.Pose2d;
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
    private final VectorF cameraOffset = new VectorF(0f, 0f, 0f);
    private final double cameraHeadingOffset = 0;

    public Vision(HardwareMap hardwareMap) {

        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.RADIANS)
                .build();

        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTagProcessor)
                .setAutoStopLiveView(true)
                .build();
    }
    public ArrayList<Pose2d> getAprilTagPoses() {
        ArrayList<AprilTagDetection> tags = new ArrayList<>();

        if (portal.getProcessorEnabled(aprilTagProcessor)) {
            tags = aprilTagProcessor.getDetections();
        }

        ArrayList<Pose2d> poses = new ArrayList<>();

        for (AprilTagDetection tag: tags) {
            if (tag.metadata != null) {

                // Get the tag absolute position on the field
                Transform3d tagPose = new Transform3d(
                        tag.metadata.fieldPosition,
                        tag.metadata.fieldOrientation
                );

                // Get the relative location of the tag from the camera
                Transform3d cameraToTagTransform = new Transform3d(
                        new VectorF(
                                (float) tag.rawPose.x,
                                (float) tag.rawPose.y,
                                (float) tag.rawPose.z
                        ),
                        Transform3d.MatrixToQuaternion(tag.rawPose.R)
                );

                // Inverse the previous transform to get the location of the camera from the tag
                Transform3d tagToCameraTransform = cameraToTagTransform.unaryMinusInverse();

                // Add the tag position and the relative position of the camera to the tag
                Transform3d cameraPose = tagPose.plus(tagToCameraTransform);

                // The the relative location of the camera to the robot
                //TODO: You have to tune this value for your camera
                Transform3d robotToCameraTransform = new Transform3d(
                        cameraOffset,
                        yawToQuaternion(cameraHeadingOffset)
                );

                // Inverse the previous transform again to get the location of the robot from the camera
                Transform3d cameraToRobotTransform = robotToCameraTransform.unaryMinusInverse();

                // Add the relative location of the robot to location of the Camera
                Transform3d robotPose = cameraPose.plus(cameraToRobotTransform);

                // Convert from a 3D transform to a 2D pose
                poses.add(robotPose.toPose2d());
            }
        }

        return poses;
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

    public static Quaternion yawToQuaternion(double yaw) {
        double halfYaw = yaw / 2.0;
        double w = Math.cos(halfYaw);
        double z = Math.sin(halfYaw);

        // Since it's only yaw, x and y are 0
        return new Quaternion((float) w, 0, 0,(float) z, System.nanoTime());
    }
}

