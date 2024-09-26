package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.robot.subsystems.Vision;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name = "AprilTag Localization New", group = "vision")
public class AprilTagLocalization extends LinearOpMode {

    // Name of the webcam in the hardware map
    private static final String WEBCAM_NAME = "Webcam 1";


    private final double xOff = -3;
    private final double yOff = -9;

    Vision vision;

    private final double headdingOff = 180;
    @Override
    public void runOpMode() {

        vision = new Vision(hardwareMap);
        FtcDashboard dashboard = FtcDashboard.getInstance();

        // Telemetry to indicate initialization is complete
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start
        waitForStart();

        if (opModeIsActive()) {
            vision.update();
            while (opModeIsActive()) {


                List<Pose2d> currentDetections = vision.getAprilTagPoses();

                if (currentDetections.isEmpty()) {
                    telemetry.addLine("No tags detected");
                } else {
                    // Iterate through the detected tags
                    for (Pose2d pose: currentDetections) {
                        telemetry.addData("Tag Pose", pose.toString());
                        telemetry.update();

                        TelemetryPacket packet = new TelemetryPacket();
                        packet.fieldOverlay().fillCircle(pose.position.x, pose.position.y, 2);
                        dashboard.sendTelemetryPacket(packet);
                    }
                }

                telemetry.addLine("Running opmode");
                telemetry.update();
                vision.update();
            }
        }

    }


}
