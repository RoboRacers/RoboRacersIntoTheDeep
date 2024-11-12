package org.firstinspires.ftc.teamcode.vision;

import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;

public class SampleOtherTeamCode extends OpenCvPipeline
{
    /*
     * Working image buffers
     */

    public double targetAngle;
    Mat ycrcbMat = new Mat();
    Mat crMat = new Mat();
    Mat cbMat = new Mat();

    Mat blueThresholdMat = new Mat();
    Mat redThresholdMat = new Mat();
    Mat yellowThresholdMat = new Mat();

    Mat morphedBlueThreshold = new Mat();
    Mat morphedRedThreshold = new Mat();
    Mat morphedYellowThreshold = new Mat();

    Mat contoursOnPlainImageMat = new Mat();

    /*
     * Threshold values
     */
    static final int YELLOW_MASK_THRESHOLD = 57;
    static final int BLUE_MASK_THRESHOLD = 150;
    static final int RED_MASK_THRESHOLD = 198;

    /*
     * Area threshold for detected objects
     */
    static final double AREA_THRESHOLD = 500.0; // Adjust this value as needed

    /*
     * Elements for noise reduction
     */
    Mat erodeElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3.5, 3.5));
    Mat dilateElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3.5, 3.5));

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
        YCrCb,
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

        if(nextStageNum >= stages.length)
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
            case YCrCb:
            {
                return ycrcbMat;
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
        // Convert the input image to YCrCb color space
        Imgproc.cvtColor(input, ycrcbMat, Imgproc.COLOR_RGB2YCrCb);

        // Extract the Cb and Cr channels
        Core.extractChannel(ycrcbMat, cbMat, 2); // Cb channel index is 2
        Core.extractChannel(ycrcbMat, crMat, 1); // Cr channel index is 1

        // Threshold the channels to form masks
        Imgproc.threshold(cbMat, blueThresholdMat, BLUE_MASK_THRESHOLD, 255, Imgproc.THRESH_BINARY);
        Imgproc.threshold(crMat, redThresholdMat, RED_MASK_THRESHOLD, 255, Imgproc.THRESH_BINARY);
        Imgproc.threshold(cbMat, yellowThresholdMat, YELLOW_MASK_THRESHOLD, 255, Imgproc.THRESH_BINARY_INV);

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
        for(MatOfPoint contour : blueContoursList)
        {
            analyzeContour(contour, input, "Blue");
        }

        for(MatOfPoint contour : redContoursList)
        {
            analyzeContour(contour, input, "Red");
        }

        for(MatOfPoint contour : yellowContoursList)
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

        double closestDistance = Double.MAX_VALUE;
        AnalyzedStone closestStone = null;

        for (AnalyzedStone stone : internalStoneList)
        {
            // Check if the stone's color matches the target color
            if (stone.color.equals(targetColor))
            {


                // Get the center of the rotated rectangle
                Point center = stone.rotatedRect.center;

                // Calculate the distance to the camera center
                double distance = Math.sqrt(Math.pow(center.x - 640, 2) + Math.pow(center.y - cameraCenter.y, 2));

                if (distance < closestDistance)
                {
                    closestDistance = distance;
                    closestStone = stone;
                }
            }
        }

        // Draw a green rectangle around the closest detected object
        if (closestStone != null)
        {
            drawRotatedRect(closestStone.rotatedRect, input, "Green");
            targetAngle = closestStone.angle;

        }
    }

    void drawRotatedRect(RotatedRect rect, Mat drawOn, String color)
    {
        Point[] points = new Point[4];
        rect.points(points);
        for (int i = 0; i < 4; i++)
        {
            Imgproc.line(drawOn, points[i], points[(i + 1) % 4], getColorScalar(color), CONTOUR_LINE_THICKNESS);
        }
    }

    static Scalar getColorScalar(String color)
    {
        switch (color)
        {
            case "Red":
                return RED;
            case "Blue":
                return BLUE;
            case "Yellow":
                return YELLOW;
            case "Green":
                return GREEN;
            default:
                return RED;
        }
    }

    void drawTagText(RotatedRect rect, String text, Mat drawOn, String color)
    {
        Point center = rect.center;
        Imgproc.putText(drawOn, text, new Point(center.x - 15, center.y - 15), Imgproc.FONT_HERSHEY_PLAIN, 0.5, new Scalar(50,50,50), 1);
    }
}
