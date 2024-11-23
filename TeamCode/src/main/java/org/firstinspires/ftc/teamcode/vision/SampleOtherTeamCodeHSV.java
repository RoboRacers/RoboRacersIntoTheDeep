package org.firstinspires.ftc.teamcode.vision;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

public class SampleOtherTeamCodeHSV extends OpenCvPipeline
{
    /*
     * Working image buffers
     */
    public double targetAngle;
    Mat hsvMat = new Mat();

    Mat blueThresholdMat = new Mat();
    Mat redThresholdMat = new Mat();
    Mat yellowThresholdMat = new Mat();

    Mat morphedBlueThreshold = new Mat();
    Mat morphedRedThreshold = new Mat();
    Mat morphedYellowThreshold = new Mat();

    Mat contoursOnPlainImageMat = new Mat();

    /*
     * Area threshold for detected objects
     */
    public static final double AREA_THRESHOLD = 500.0; // Adjust this value as needed

    /*
     * Elements for noise reduction
     */
    Mat erodeElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
    Mat dilateElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));

    /*
     * Colors
     */
    static final Scalar RED = new Scalar(255, 0, 0);
    static final Scalar BLUE = new Scalar(0, 0, 255);
    static final Scalar YELLOW = new Scalar(255, 255, 0);
    static final Scalar GREEN = new Scalar(0, 255, 0);

    static final int CONTOUR_LINE_THICKNESS = 2;

    static class AnalyzedStone
    {
        double angle;
        String color;
        RotatedRect rotatedRect; // Store the rotated rectangle for drawing
    }

    ArrayList<AnalyzedStone> internalStoneList = new ArrayList<>();
    volatile ArrayList<AnalyzedStone> clientStoneList = new ArrayList<>();

    /*
     * Viewport stages
     */
    enum Stage
    {
        FINAL,
        HSV,
        MASKS,
        MASKS_NR,
        CONTOURS;
    }

    Stage[] stages = Stage.values();
    int stageNum = 0;

    // Variable to specify which color's closest object to highlight in green
    String targetColor = "Red"; // Change this to "Red", "Blue", or "Yellow"

    @Override
    public void onViewportTapped()
    {
        int nextStageNum = stageNum + 1;

        if (nextStageNum >= stages.length)
        {
            nextStageNum = 0;
        }

        stageNum = nextStageNum;
    }

    @Override
    public Mat processFrame(Mat input)
    {
        internalStoneList.clear();

        /*
         * Run the image processing
         */
        findContours(input);

        clientStoneList = new ArrayList<>(internalStoneList);

        /*
         * Decide which buffer to send to the viewport
         */
        switch (stages[stageNum])
        {
            case HSV:
            {
                return hsvMat;
            }

            case FINAL:
            {
                return input;
            }

            case MASKS:
            {
                Mat masks = new Mat();
                Core.addWeighted(yellowThresholdMat, 1.0, redThresholdMat, 1.0, 0.0, masks);
                Core.addWeighted(masks, 1.0, blueThresholdMat, 1.0, 0.0, masks);
                return masks;
            }

            case MASKS_NR:
            {
                Mat masksNR = new Mat();
                Core.addWeighted(morphedYellowThreshold, 1.0, morphedRedThreshold, 1.0, 0.0, masksNR);
                Core.addWeighted(masksNR, 1.0, morphedBlueThreshold, 1.0, 0.0, masksNR);
                return masksNR;
            }

            case CONTOURS:
            {
                return contoursOnPlainImageMat;
            }

            default:
            {
                return input;
            }
        }
    }

    public ArrayList<AnalyzedStone> getDetectedStones()
    {
        return clientStoneList;
    }

    void findContours(Mat input)
    {
        // Convert the input image to HSV color space
        Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGB2HSV);

        // Threshold for blue color in HSV
        Core.inRange(hsvMat, new Scalar(100, 150, 50), new Scalar(140, 255, 255), blueThresholdMat); // lower (90,150,100) and upper is (130,255,255)

        // Threshold for red color in HSV (two ranges for red in HSV)
        Mat lowerRed = new Mat();
        Mat upperRed = new Mat();
        Core.inRange(hsvMat, new Scalar(0, 120, 70), new Scalar(10, 255, 255), lowerRed);
        Core.inRange(hsvMat, new Scalar(170, 120, 70), new Scalar(180, 255, 255), upperRed); // could also be (170, 100, 100) for red
        Core.addWeighted(lowerRed, 1.0, upperRed, 1.0, 0.0, redThresholdMat);

        // Threshold for yellow color in HSV
        Core.inRange(hsvMat, new Scalar(20, 150, 50), new Scalar(30, 255, 255), yellowThresholdMat); // lower is (20,100,100) and upper is fine

        // Apply morphology to the masks
        morphMask(blueThresholdMat, morphedBlueThreshold);
        morphMask(redThresholdMat, morphedRedThreshold);
        morphMask(yellowThresholdMat, morphedYellowThreshold);

        // Find contours in the masks
        ArrayList<MatOfPoint> blueContoursList = new ArrayList<>();
        Imgproc.findContours(morphedBlueThreshold, blueContoursList, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_NONE);

        ArrayList<MatOfPoint> redContoursList = new ArrayList<>();
        Imgproc.findContours(morphedRedThreshold, redContoursList, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_NONE);

        ArrayList<MatOfPoint> yellowContoursList = new ArrayList<>();
        Imgproc.findContours(morphedYellowThreshold, yellowContoursList, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_NONE);

        // Create a plain image for drawing contours
        contoursOnPlainImageMat = Mat.zeros(input.size(), input.type());

        // Analyze and draw contours
        for (MatOfPoint contour : blueContoursList)
        {
            analyzeContour(contour, input, "Blue");
        }

        for (MatOfPoint contour : redContoursList)
        {
            analyzeContour(contour, input, "Red");
        }

        for (MatOfPoint contour : yellowContoursList)
        {
            analyzeContour(contour, input, "Yellow");
        }

        // Draw the closest detected object of the target color in green
        drawClosestObject(input);
    }

    void morphMask(Mat input, Mat output)
    {
        /*
         * Apply erosion and dilation for noise reduction
         */
        Imgproc.erode(input, output, erodeElement);
        Imgproc.erode(output, output, erodeElement);

        Imgproc.dilate(output, output, dilateElement);
        Imgproc.dilate(output, output, dilateElement);
    }

    void analyzeContour(MatOfPoint contour, Mat input, String color)
    {
        // Transform the contour to a different format
        Point[] points = contour.toArray();
        MatOfPoint2f contour2f = new MatOfPoint2f(points);

        // Calculate the area of the contour
        double area = Imgproc.contourArea(contour);

        // Check if the area exceeds the threshold
        if (area > AREA_THRESHOLD)
        {
            // Fit a rotated rectangle to the contour and draw it
            RotatedRect rotatedRectFitToContour = Imgproc.minAreaRect(contour2f);
            drawRotatedRect(rotatedRectFitToContour, input, color);
            drawRotatedRect(rotatedRectFitToContour, contoursOnPlainImageMat, color);

            // Adjust the angle based on rectangle dimensions
            double rotRectAngle = rotatedRectFitToContour.angle;
            if (rotatedRectFitToContour.size.width < rotatedRectFitToContour.size.height)
            {
                rotRectAngle += 90;
            }

            // Compute the angle and store it
            double angle = -(rotRectAngle - 180);
            drawTagText(rotatedRectFitToContour, Integer.toString((int) Math.round(angle)) + " deg", input, color);

            // Store the detected stone information
            AnalyzedStone analyzedStone = new AnalyzedStone();
            analyzedStone.angle = -(rotRectAngle - 180);
            analyzedStone.color = color;
            analyzedStone.rotatedRect = rotatedRectFitToContour; // Store the rotated rectangle
            internalStoneList.add(analyzedStone);
        }
    }

    void drawClosestObject(Mat input)
    {
        if (internalStoneList.isEmpty()) return;

        // Get the center of the camera view
        Point cameraCenter = new Point(input.width() / 2.0, input.height() / 2.0);

        double minDistance = Double.MAX_VALUE;
        AnalyzedStone closestObject = null;

        for (AnalyzedStone stone : internalStoneList)
        {
            if (stone.color.equals(targetColor))
            {
                double distance = Math.abs(stone.rotatedRect.center.x - cameraCenter.x);
                if (distance < minDistance)
                {
                    closestObject = stone;
                    minDistance = distance;
                }
            }
        }

        if (closestObject != null)
        {
            drawRotatedRect(closestObject.rotatedRect, input, "Green");
        }
    }

    void drawRotatedRect(RotatedRect rect, Mat drawOn, String color)
    {
        Point[] points = new Point[4];
        rect.points(points);

        Scalar s;
        if (color.equals("Red")) s = RED;
        else if (color.equals("Blue")) s = BLUE;
        else if (color.equals("Yellow")) s = YELLOW;
        else if (color.equals("Green")) s = GREEN;
        else s = RED;

        for (int i = 0; i < 4; ++i)
        {
            Imgproc.line(drawOn, points[i], points[(i + 1) % 4], s, CONTOUR_LINE_THICKNESS);
        }
    }

    void drawTagText(RotatedRect rect, String text, Mat drawOn, String color)
    {
        Scalar s;
        if (color.equals("Red")) s = RED;
        else if (color.equals("Blue")) s = BLUE;
        else if (color.equals("Yellow")) s = YELLOW;
        else if (color.equals("Green")) s = GREEN;
        else s = RED;

        Imgproc.putText(drawOn, text, rect.center, Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, s, 1);
    }
}
