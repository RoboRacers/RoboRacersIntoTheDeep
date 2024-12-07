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
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

class HSVBasedPipeline extends OpenCvPipeline {
    private double targetAngle = 0;
    private int detectedObjectsCount = 0;

    private final Scalar lowerYellow = new Scalar(20, 100, 100);
    private final Scalar upperYellow = new Scalar(30, 255, 255);

    private final Scalar lowerBlue = new Scalar(100, 150, 50);
    private final Scalar upperBlue = new Scalar(130, 255, 255);

    private final Scalar lowerRed1 = new Scalar(0, 120, 70);
    private final Scalar upperRed1 = new Scalar(10, 255, 240);

    private final Scalar lowerRed2 = new Scalar(170, 120, 70);
    private final Scalar upperRed2 = new Scalar(180, 255, 240);

    private final Mat hsvFrame = new Mat();
    private final Mat blueMask = new Mat();
    private final Mat redMask = new Mat();
    private final Mat yellowCandidateMask = new Mat();
    private final Mat combinedMask = new Mat();
    private final Mat processedMask = new Mat();
    private final Mat hierarchy = new Mat();

    private final List<MatOfPoint> contours = new ArrayList<>();

    // Adjustable parameter for minimum contour area to filter noise
    private static final double MIN_CONTOUR_AREA = 500.0;

    @Override
    public Mat processFrame(Mat input) {
        // Convert the frame to HSV
        Imgproc.cvtColor(input, hsvFrame, Imgproc.COLOR_BGR2HSV);

        // Apply color thresholds for blue
        Core.inRange(hsvFrame, lowerBlue, upperBlue, blueMask);

        // Apply color thresholds for red
        Mat lowerRedMask = new Mat();
        Mat upperRedMask = new Mat();
        Core.inRange(hsvFrame, lowerRed1, upperRed1, lowerRedMask);
        Core.inRange(hsvFrame, lowerRed2, upperRed2, upperRedMask);
        Core.addWeighted(lowerRedMask, 1.0, upperRedMask, 1.0, 0.0, redMask);

        // Detect areas that are not blue or red (potential yellow candidates)
        Mat nonBlueRedMask = new Mat();
        Core.bitwise_not(blueMask, nonBlueRedMask); // Invert blue mask
        Core.bitwise_not(redMask, nonBlueRedMask, nonBlueRedMask); // Subtract red areas

        // Yellow candidates: regions that are non-blue, non-red, and meet HSV yellow thresholds
        Core.inRange(hsvFrame, lowerYellow, upperYellow, yellowCandidateMask);
        Core.bitwise_and(yellowCandidateMask, nonBlueRedMask, yellowCandidateMask);

        // Combine masks for final processing
        Core.addWeighted(blueMask, 1.0, redMask, 1.0, 0.0, combinedMask);
        Core.addWeighted(combinedMask, 1.0, yellowCandidateMask, 1.0, 0.0, combinedMask);

        // Apply morphological operations to reduce noise
        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
        Imgproc.morphologyEx(combinedMask, processedMask, Imgproc.MORPH_OPEN, kernel);

        // Find contours
        contours.clear();
        Imgproc.findContours(processedMask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        // Filter and count valid contours
        detectedObjectsCount = 0;
        MatOfPoint largestContour = null;
        double largestArea = 0;

        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            if (area > MIN_CONTOUR_AREA) {
                detectedObjectsCount++;
                if (area > largestArea) {
                    largestArea = area;
                    largestContour = contour;
                }
            }
        }

        // Process the largest valid contour (if any)
        if (largestContour != null) {
            Rect boundingBox = Imgproc.boundingRect(largestContour);
            Point center = new Point(
                    boundingBox.x + boundingBox.width / 2.0,
                    boundingBox.y + boundingBox.height / 2.0
            );

            double frameCenterX = input.width() / 2.0;
            targetAngle = Math.atan2(center.x - frameCenterX, input.height());

            // Draw the bounding box and center point
            Imgproc.rectangle(input, boundingBox, new Scalar(0, 255, 0), 2);
            Imgproc.circle(input, center, 5, new Scalar(255, 0, 0), -1);
        }

        // Return the processed frame for display
        return input;
    }

    public double getTargetAngle() {
        return targetAngle;
    }

    public int getDetectedObjectsCount() {
        return detectedObjectsCount;
    }
}