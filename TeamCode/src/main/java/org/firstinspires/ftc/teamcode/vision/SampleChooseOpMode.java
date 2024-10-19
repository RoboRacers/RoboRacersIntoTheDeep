package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "Sample Closest Object OpMode", group = "Auto")
public class SampleChooseOpMode extends LinearOpMode {
    OpenCvCamera webcam;
    SampleDetectDataPipeline pipeline;

    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        pipeline = new SampleDetectDataPipeline();
        webcam.setPipeline(pipeline);

        // Start the camera streaming
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addLine("Camera could not be opened.");
            }
        });

        telemetry.addLine("Waiting for start");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");

            // Make a copy of the detected objects list to avoid concurrent modification
            List<SampleDetectDataPipeline.DetectedObject> detectedObjectsCopy = new ArrayList<>(pipeline.detectedObjects);

            // Find the closest object based on the largest area (proxy for distance)
            SampleDetectDataPipeline.DetectedObject closestObject = null;
            double maxArea = 0;

            // Validate detected objects
            for (SampleDetectDataPipeline.DetectedObject obj : detectedObjectsCopy) {
                if (isValidObject(obj)) {
                    // Calculate the area using the object's corners
                    double area = calculateArea(obj.corners);
                    if (area > maxArea) {
                        maxArea = area;
                        closestObject = obj;
                    }
                }
            }

            if (closestObject != null) {
                telemetry.addData("Closest Object Color", closestObject.color);
                telemetry.addData("Closest Object Center", closestObject.center);
                telemetry.addData("Closest Object Area", maxArea);
            } else {
                telemetry.addLine("No valid objects detected.");
            }

            telemetry.update();
        }

        webcam.stopStreaming();
    }

    // Helper function to calculate the area of the detected object's bounding box
    public double calculateArea(Point[] corners) {
        // Calculate the area of the rotated rectangle using its corner points
        double width = Math.sqrt(Math.pow(corners[0].x - corners[1].x, 2) + Math.pow(corners[0].y - corners[1].y, 2));
        double height = Math.sqrt(Math.pow(corners[1].x - corners[2].x, 2) + Math.pow(corners[1].y - corners[2].y, 2));
        return width * height;
    }

    // Function to validate that the detected object is valid
    public boolean isValidObject(SampleDetectDataPipeline.DetectedObject obj) {
        // Check if the object is null
        if (obj == null) {
            telemetry.addLine("Detected object is null.");
            return false;
        }

        // Check if the corners array is valid (not null and has exactly 4 points)
        if (obj.corners == null || obj.corners.length != 4) {
            telemetry.addLine("Invalid corner data.");
            return false;
        }

        // Ensure each corner is valid and within reasonable bounds (camera resolution: 640x480)
        for (Point corner : obj.corners) {
            if (corner == null || corner.x < 0 || corner.x > 640 || corner.y < 0 || corner.y > 480) {
                telemetry.addLine("Corner point is invalid.");
                return false;
            }
        }

        return true; // Object is valid
    }
}
