package org.firstinspires.ftc.teamcode.vision;

import org.firstinspires.ftc.teamcode.vision.HSVBasedPipeline;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "HSV Vision OpMode", group = "Vision")
public class HSVBased extends LinearOpMode {
    OpenCvCamera camera;
    HSVBasedPipeline pipeline;

//    CRServo rotateClaw;
//    Servo claw;

    @Override
    public void runOpMode() throws InterruptedException {
//        claw = hardwareMap.get(Servo.class, "claw");
//        rotateClaw = hardwareMap.get(CRServo.class, "rotateClaw");

        // Initialize the camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        // Initialize the pipeline
        pipeline = new HSVBasedPipeline();
        camera.setPipeline(pipeline);

        // Open the camera
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Error", errorCode);
                telemetry.update();
            }
        });

        // Wait for the start button to be pressed
        waitForStart();

        while (opModeIsActive()) {
            // Get target angle and detected objects count
            double targetAngle = pipeline.getTargetAngle();
            int detectedObjects = pipeline.getDetectedObjectsCount();

            // Display telemetry
            telemetry.addData("Target Angle (Radians)", targetAngle);
//            telemetry.addData("Detected Objects", detectedObjects);
//            telemetry.addData("Claw Position", claw.getPosition());
//            telemetry.addData("Rotate Claw Direction", rotateClaw.getDirection());
            telemetry.addData("Detected Objects", pipeline.getDetectedObjectsCount());
            telemetry.update();

            // Reduce CPU usage
            sleep(100);
        }

        // Stop the camera when the OpMode ends
        camera.stopStreaming();
    }
}
