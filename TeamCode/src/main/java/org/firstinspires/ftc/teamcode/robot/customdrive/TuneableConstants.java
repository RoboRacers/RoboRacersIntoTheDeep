package org.firstinspires.ftc.teamcode.robot.customdrive;

import com.acmerobotics.dashboard.config.Config;
import com.roboracers.topgear.controls.PIDCoefficients;
import com.roboracers.topgear.follower.CentripetalGuidedVectorFieldFollower;

@Config
public class TuneableConstants {

    /**
     * The PID coefficients for the x controller.
     */
    public static PIDCoefficients X_PID_COEFFS = new PIDCoefficients(0.07,0,0.05);
    /**
     * The PID coefficients for the y controller.
     */
    public static PIDCoefficients Y_PID_COEFFS = new PIDCoefficients(0.01,0,0.05);
    /**
     * The PID coefficients for the heading controller.
     */
    public static PIDCoefficients H_PID_COEFFS = new PIDCoefficients(0,0,0);
    /**
     * The distance between the closest point and the tangent point, measured in inches.
     */
    public static double TANGENT_DISTANCE = 3;
    /**
     * Used to compute centripetal force.
     * Not the literal mass of the robot, but a constant used in the calculation.
     */
    public static double CENTRIPETAL_MASS = 0.07;
    /**
     * Max speed of the robot while following the path.
     * Measured between 0 and 1.
     */
    public static double GVF_FOLLOWING_MAX_SPEED = 1;
    /**
     * Threshold for the end PID to kick in, measured in inches.
     */
    public static double PID_FOLLOWING_THRESHOLD = 2;
    /**
     * The distance threshold for the end of the path, measured in inches.
     */
    public static double STOPPING_DISTANCE_THRESHOLD = 3;
    /**
     * The minimum power for the end of the path, measured between 0 and 1.
     */
    public static double STOPPING_POWER_THRESHOLD = 0.1;


    public static CentripetalGuidedVectorFieldFollower.Params getParams() {
        return new CentripetalGuidedVectorFieldFollower.Params(
                TANGENT_DISTANCE,
                CENTRIPETAL_MASS,
                GVF_FOLLOWING_MAX_SPEED,
                PID_FOLLOWING_THRESHOLD,
                STOPPING_DISTANCE_THRESHOLD,
                STOPPING_POWER_THRESHOLD,
                X_PID_COEFFS,
                Y_PID_COEFFS,
                H_PID_COEFFS
        );
    }


}
