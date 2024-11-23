package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
import java.util.List;

class HSVBasedPipeline extends OpenCvPipeline {
    private String targetColor = "Blue"; // Default color
    private double targetAngle = 0;
    private int detectedObjectsCount = 0;

    private final Scalar lowerYellow = new Scalar(20, 100, 100);
    private final Scalar upperYellow = new Scalar(30, 255, 255);

    private final Scalar lowerBlue = new Scalar(100, 150, 100);
    private final Scalar upperBlue = new Scalar(140, 255, 255);

    private final Scalar lowerRed1 = new Scalar(0, 120, 100);
    private final Scalar upperRed1 = new Scalar(10, 255, 255);
    private final Scalar lowerRed2 = new Scalar(160, 120, 100);
    private final Scalar upperRed2 = new Scalar(180, 255, 255);

    private final Mat hsvFrame = new Mat();
    private final Mat mask = new Mat();
    private final Mat hierarchy = new Mat();

    private final List<MatOfPoint> contours = new ArrayList<>();

    @Override
    public Mat processFrame(Mat input) {
        // Convert the frame to HSV
        Imgproc.cvtColor(input, hsvFrame, Imgproc.COLOR_BGR2HSV);

        // Apply color threshold based on the target color
        switch (targetColor) {
            case "Yellow":
                Core.inRange(hsvFrame, lowerYellow, upperYellow, mask);
                break;
            case "Blue":
                Core.inRange(hsvFrame, lowerBlue, upperBlue, mask);
                break;
            case "Red":
                Mat lowerRedMask = new Mat();
                Mat upperRedMask = new Mat();
                Core.inRange(hsvFrame, lowerRed1, upperRed1, lowerRedMask);
                Core.inRange(hsvFrame, lowerRed2, upperRed2, upperRedMask);
                Core.addWeighted(lowerRedMask, 1.0, upperRedMask, 1.0, 0.0, mask);
                lowerRedMask.release();
                upperRedMask.release();
                break;
        }

        // Find contours
        contours.clear();
        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        // Find the largest contour (if any)
        detectedObjectsCount = contours.size();
        if (!contours.isEmpty()) {
            MatOfPoint largestContour = contours.stream()
                    .max((c1, c2) -> Double.compare(Imgproc.contourArea(c1), Imgproc.contourArea(c2)))
                    .orElse(null);

            if (largestContour != null) {
                // Calculate the bounding box and center point
                Rect boundingBox = Imgproc.boundingRect(largestContour);
                Point center = new Point(
                        boundingBox.x + boundingBox.width / 2.0,
                        boundingBox.y + boundingBox.height / 2.0
                );

                // Calculate the angle (relative to the center of the frame)
                double frameCenterX = input.width() / 2.0;
                targetAngle = Math.atan2(center.x - frameCenterX, input.height());

                // Draw the bounding box and center point
                Imgproc.rectangle(input, boundingBox, new Scalar(0, 255, 0), 2);
                Imgproc.circle(input, center, 5, new Scalar(255, 0, 0), -1);
            }
        }

        // Return the processed frame for display
        return input;
    }

    public void setTargetColor(String color) {
        targetColor = color;
    }

    public String getTargetColor() {
        return targetColor;
    }

    public double getTargetAngle() {
        return targetAngle;
    }

    public int getDetectedObjectsCount() {
        return detectedObjectsCount;
    }
}