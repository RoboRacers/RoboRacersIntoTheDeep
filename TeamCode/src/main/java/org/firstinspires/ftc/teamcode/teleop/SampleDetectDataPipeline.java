package org.firstinspires.ftc.teamcode.teleop;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class SampleDetectDataPipeline extends OpenCvPipeline {
    static final int YELLOW_MASK_THRESHOLD = 55;
    static final int BLUE_MASK_THRESHOLD = 150;
    static final int RED_MASK_THRESHOLD = 200;
    static final double MIN_AREA_THRESHOLD = 500;
    static final double MAX_AREA_THRESHOLD = 20000;

    public static class DetectedObject {
        public String color;
        public Point center;
        public Point[] corners;

        public DetectedObject(String color, Point center, Point[] corners) {
            this.color = color;
            this.center = center;
            this.corners = corners;
        }
    }

    public List<DetectedObject> detectedObjects = new ArrayList<>();

    @Override
    public Mat processFrame(Mat input) {
        Mat ycrcbImage = new Mat();
        Mat cbMat = new Mat();
        Mat crMat = new Mat();
        Mat yellowMask = new Mat();
        Mat blueMask = new Mat();
        Mat redMask = new Mat();

        // Clear previously detected objects
        detectedObjects.clear();

        // Convert to YCrCb color space
        Imgproc.cvtColor(input, ycrcbImage, Imgproc.COLOR_BGR2YCrCb);

        // Extract Cb and Cr channels
        Core.extractChannel(ycrcbImage, cbMat, 2); // Cb channel
        Core.extractChannel(ycrcbImage, crMat, 1); // Cr channel

        // Threshold for colors
        Imgproc.threshold(cbMat, blueMask, BLUE_MASK_THRESHOLD, 255, Imgproc.THRESH_BINARY);
        Imgproc.threshold(crMat, redMask, RED_MASK_THRESHOLD, 255, Imgproc.THRESH_BINARY);
        Imgproc.threshold(cbMat, yellowMask, YELLOW_MASK_THRESHOLD, 255, Imgproc.THRESH_BINARY_INV);

        // Detect and draw contours for each color
        detectAndDrawContours(input, yellowMask, new Scalar(0, 255, 0), "Yellow"); // Yellow contours
        detectAndDrawContours(input, blueMask, new Scalar(255, 0, 0), "Blue");      // Blue contours
        detectAndDrawContours(input, redMask, new Scalar(0, 0, 255), "Red");        // Red contours

        return input;
    }

    public void detectAndDrawContours(Mat originalImage, Mat mask, Scalar color, String colorName) {
        // Ensure the mask is binary and of type CV_8U
        if (mask.type() != CvType.CV_8U) {
            mask.convertTo(mask, CvType.CV_8U);
        }

        // Find contours
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        // Draw contours and process for angle detection
        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            if (area < MIN_AREA_THRESHOLD || area > MAX_AREA_THRESHOLD) {
                continue; // Skip if area is not in the expected range
            }

            // Draw the contour
            Imgproc.drawContours(originalImage, contours, contours.indexOf(contour), color, 2);

            // Process the contour for angle detection
            MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());
            RotatedRect rotatedRect = Imgproc.minAreaRect(contour2f);
            drawRotatedRect(rotatedRect, originalImage, color);

            // Get the center and corner points of the rectangle
            Point[] corners = new Point[4];
            rotatedRect.points(corners);
            Point center = rotatedRect.center;

            // Draw the center point on the image
            Imgproc.circle(originalImage, center, 5, color, -1);

            // Add detected object (color, center, and corners) to the list
            detectedObjects.add(new DetectedObject(colorName, center, corners));

            // Draw corner points on the image
            for (Point corner : corners) {
                Imgproc.circle(originalImage, corner, 5, color, -1);
            }

            // Get and draw the angle of the rectangle
            double angle = rotatedRect.angle;
            if (rotatedRect.size.width < rotatedRect.size.height) {
                angle += 90;
            }
            Imgproc.putText(originalImage, Math.round(angle) + " deg",
                    new Point(rotatedRect.center.x - 50, rotatedRect.center.y + 25),
                    Imgproc.FONT_HERSHEY_PLAIN, 1.5, color, 2);
        }
    }

    public void drawRotatedRect(RotatedRect rect, Mat drawOn, Scalar color) {
        Point[] points = new Point[4];
        rect.points(points);

        for (int i = 0; i < 4; ++i) {
            Imgproc.line(drawOn, points[i], points[(i + 1) % 4], color, 2);
        }
    }
}
