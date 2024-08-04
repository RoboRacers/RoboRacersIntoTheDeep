package org.firstinspires.ftc.teamcode.robot.customdrive;

import com.acmerobotics.dashboard.config.Config;
import com.roboracers.pathfollower.controls.PIDCoefficients;

@Config
public class TuneableConstants {

    /*
     * The PID coefficients for the x, y, and heading controllers, used to bring the robot to a stop at the end of GVF following.
     */
    /**
     * The PID coefficients for the x controller.
     */
    public static PIDCoefficients xPIDCoeffs = new PIDCoefficients(0.07,0,0.05);
    /**
     * The PID coefficients for the y controller.
     */
    public static PIDCoefficients yPIDCoeffs = new PIDCoefficients(0.01,0,0.05);
    /**
     * The PID coefficients for the heading controller.
     */
    public static PIDCoefficients headingPIDCoeffs = new PIDCoefficients(0,0,0);
    /**
     * The distance between the closest point and the tangent point, measured in inches.
     */
    public static double tangentDistance = 3;
    /**
     * Max speed of the robot while following the path.
     * Measured between 0 and 1.
     */
    public static double maxSpeed = 0.65;
    /**
     * Threshold for the end PID to kick in, measured in inches.
     */
    public static double PIDThreshold = 2;

    /**
     * The default curvature for the bezier curve builder..
     */
    public static double DEFAULT_CURVATURE = 0.5;



}
