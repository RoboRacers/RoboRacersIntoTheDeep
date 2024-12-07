package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@TeleOp(name = "Enhanced HSV and Angle Detection OpMode", group = "Vision")
public class EnhancedHSVAndAngleDetection extends LinearOpMode {
    OpenCvCamera camera;
    CombinedHSVAndAnglePipeline pipeline;

    @Override
    public void runOpMode() {
        // Initialize the camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        // Initialize the pipeline
        pipeline = new CombinedHSVAndAnglePipeline();
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

        waitForStart();

        while (opModeIsActive()) {
            // Get target angle and detected objects count
            double targetAngle = pipeline.getTargetAngle();
            int detectedObjects = pipeline.getDetectedObjectsCount();

            // Display telemetry
            telemetry.addData("Target Angle (Radians)", targetAngle);
            telemetry.addData("Detected Objects", detectedObjects);
            telemetry.update();

            sleep(100); // Reduce CPU usage
        }

        camera.stopStreaming();
    }
}
