package pathfollower.follower;

import com.roboracers.pathfollower.controls.PIDCoefficients;
import com.roboracers.pathfollower.controls.PIDController;
import com.roboracers.pathfollower.follower.Follower;
import com.roboracers.pathfollower.geometry.PointProjection;
import com.roboracers.pathfollower.geometry.Pose2d;
import com.roboracers.pathfollower.geometry.Vector2d;
import com.roboracers.pathfollower.planner.ParametricPath;

/**
 * The GuidedVectorFieldFollower class implements a robot controller that follows a guided vector field (GVF).
 * This class provides methods to set the desired vector field and update the robot's position
 * based on the GVF.
 * <p>
 * GVF is a technique used in robotics for path following where the robot follows a vector field
 * that guides it along a desired path. This class handles the computation of control signals
 * to adjust the robot's movement to stay on the path defined by the GVF.
 * </p>
 */
public class GuidedVectorFieldFollower implements Follower {

    /**
     * Current parametrically defined path that is being follower.
     */
    private ParametricPath parametricPath;
    /**
     * The distance between the closest point and the tangent point.
     */
    private double tangentDistance;
    /**
     * Max speed of the robot while following the path.
     */
    private double maxSpeed = 0.75;
    /**
     * Threshold for the end PID to kick in, measured in inches.
     */
    private double PIDThreshold = 1;

    private boolean followingSequence = false;
    /*
     Ending PIDs
     */
    PIDController xPID;
    PIDController yPID;
    PIDController headingPID;

    public GuidedVectorFieldFollower(double tangentDistance) {
        if (tangentDistance <= 0)
            this.tangentDistance = 0.1;
        else
            this.tangentDistance = tangentDistance;

        xPID = new PIDController(1,0,0);
        yPID = new PIDController(1,0,0);
        headingPID = new PIDController(1,0,0);
    }

    public GuidedVectorFieldFollower(double tangentDistance, PIDCoefficients xPIDCoeffs, PIDCoefficients yPIDCoeffs, PIDCoefficients headingPIDCoeffs, double maxSpeed) {
        if (tangentDistance <= 0)
            this.tangentDistance = 0.1;
        else
            this.tangentDistance = tangentDistance;

        xPID = new PIDController(xPIDCoeffs);
        yPID = new PIDController(yPIDCoeffs);
        headingPID = new PIDController(headingPIDCoeffs);
    }

    public GuidedVectorFieldFollower(double tangentDistance, double maxSpeed) {
        if (tangentDistance <= 0)
            this.tangentDistance = 0.1;
        else
            this.tangentDistance = tangentDistance;

        this.maxSpeed = maxSpeed;

        xPID = new PIDController(1,0,0);
        yPID = new PIDController(1,0,0);
        headingPID = new PIDController(0,0,0);
    }

    /**
     * Set the current path to be followed.
     * @param parametricPath
     */
    @Override
    public void setPath(ParametricPath parametricPath) {
        this.parametricPath = parametricPath;
    }

    /**
     * Feeds the drive powers to the drivetrain based on the direction of the
     * vector gradient field at the current point. Only provides x and y translation,
     * no heading in this implementation.
     * @param currentPosition robot current position
     * @return Drive power
     */
    @Override
    public Pose2d getDriveVelocity(Pose2d currentPosition) {

        // If no path has been set, do not return anything
        if (this.parametricPath == null)
            return null;

        double threshold = 1;

        Vector2d endpoint = parametricPath.getPoint(1);
        double distanceToEnd = currentPosition.vec().distanceTo(endpoint);

        // Use the ending PID while within threshold.
        if (Math.abs(distanceToEnd) < PIDThreshold) {
            Vector2d endDerivative = parametricPath.getDerivative(0.99);

            double headingTarget = Math.atan2(endDerivative.getY(), endDerivative.getX());

            // Debugging values
            usingPID = true;
            currentClosestTValue = 0;
            currentDistanceToEnd = distanceToEnd;
            currentClosestPoint = new Vector2d(0,0);
            currentTangentPoint = new Vector2d(0,0);
            currentDrivePower = new Pose2d();
            currentHeadingTarget = headingTarget;

            return PIDToPoint(
                    new Pose2d(endpoint, headingTarget),
                    currentPosition
            );
        } else {
            usingPID = false;
            Vector2d currentPoint = currentPosition.vec();

            // Find the closest point on the path from the robot and get its t-value
            double closestTValue = PointProjection.projectionBinarySearch(parametricPath, currentPoint, 10);

            // Calculate the tangent point (point that the robot goes towards
            Vector2d tangentPoint = parametricPath.getPoint(closestTValue).add(
                    parametricPath.getDerivative(closestTValue).normalize().multiply(tangentDistance));

            // Get the vector pointing from the robot to the tangent point
            Vector2d connectingVector = tangentPoint.subtract(currentPoint);
            Vector2d normalizedVector = connectingVector.normalize();

            // Scale the speed by the max speed
            Vector2d velocityVector = normalizedVector.scalarMultiply(maxSpeed);

            // Rotate the vector to the robot's frame of reference
            Vector2d robotFrame = velocityVector.rotated(-currentPosition.getHeading());

            // Heading calcs
            double headingTarget = Math.atan2(robotFrame.getY(), robotFrame.getX());
            headingPID.setSetpoint(headingTarget);

            Pose2d drivePower = new Pose2d(robotFrame, headingPID.update(currentPosition.getHeading()));

            // Debugging values
            usingPID = false;
            currentClosestTValue = closestTValue;
            currentDistanceToEnd = distanceToEnd;
            currentClosestPoint = parametricPath.getPoint(closestTValue);
            currentTangentPoint = tangentPoint;
            currentDrivePower = drivePower;
            currentHeadingTarget = headingTarget;

            // Return the new vector in the robot's frame of reference
            return drivePower;
        }
    }

    /**
     * Feeds the drive powers to the drivetrain based on the direction of the
     * vector gradient field at the current point. Only provides x and y translation,
     * no heading in this implementation.
     * @param currentPosition robot current position
     * @return Drive power
     */
    public Pose2d getDriveVelocity2(Pose2d currentPosition) {

        // If no path has been set, do not return anything
        if (this.parametricPath == null)
            return null;

        double distanceToEnd = currentPosition.vec().distanceTo(parametricPath.getPoint(1));

        Vector2d currentPoint = currentPosition.vec();

        // Find the closest point on the path from the robot and get it's t-value
        double closestTValue = PointProjection.projectionBinarySearch(parametricPath, currentPoint, 10);

        // Calculate the tangent point (point that the robot goes towards
        Vector2d tangentPoint = parametricPath.getPoint(closestTValue).add(
                parametricPath.getDerivative(closestTValue).normalize().multiply(tangentDistance));

        // Get the vector pointing from the robot to the tangent point
        Vector2d connectingVector = tangentPoint.subtract(currentPoint);
        Vector2d normalizedVector = connectingVector.normalize();

        // Scale the speed by the max speed
        Vector2d velocityVector = normalizedVector.scalarMultiply(maxSpeed);

        Vector2d robotFrame = velocityVector.fieldToRobotCentric(currentPosition.getHeading());
        // Return the new vector in the robot's frame of reference
        return new Pose2d(robotFrame, 0);
    }

    /**
     * Check if the robot has reached the end of the path.
     * @param currentPosition robot current position
     * @return true if the robot has reached the end of the path, false otherwise
     */
    @Override
    public Boolean isComplete(Pose2d currentPosition) {

        Vector2d endpoint = parametricPath.getPoint(1);

        double delta =  currentPosition.vec().distanceTo(endpoint);

        // TODO: Make this a tunable constant
        double distanceThreshold = 0.1;

        if (delta < distanceThreshold)
            return  true;
        else
            return false;
    }

    /**
     * PID to point implementation to bring the robot to a stop.
     * @param target
     * @param currentPose
     * @return
     */
    private Pose2d PIDToPoint(Pose2d target, Pose2d currentPose) {

        xPID.setSetpoint(target.getX());
        yPID.setSetpoint(target.getY());
        headingPID.setSetpoint(target.getHeading());

        Vector2d translationPowers = new Vector2d(
                xPID.update(currentPose.getX()),
                yPID.update(currentPose.getY())
        ).rotated(-currentPose.getHeading());

        return new Pose2d(
            translationPowers,
            headingPID.update(currentPose.getHeading())
        );
    }

    /*
     * Status variables.
     */
    /**
     * Displays if the ending PID has kicked in yet or not.
     */
    protected boolean usingPID = false;
    public double currentClosestTValue;
    public double currentDistanceToEnd;
    public Vector2d currentClosestPoint;
    public Vector2d currentTangentPoint;
    public Pose2d currentDrivePower;
    public double currentHeadingTarget;

    public class DebugPacket {
        public boolean usingPID;
        public double currentClosestTValue;
        public double currentDistanceToEnd;
        public Vector2d currentClosestPoint;
        public Vector2d currentTangentPoint;
        public Pose2d currentDrivePower;
        public double currentHeadingTarget;

        public DebugPacket(GuidedVectorFieldFollower follower) {
            this.usingPID = follower.usingPID;
            this.currentClosestTValue = follower.currentClosestTValue;
            this.currentDistanceToEnd = follower.currentDistanceToEnd;
            this.currentClosestPoint = follower.currentClosestPoint;
            this.currentTangentPoint = follower.currentTangentPoint;
            this.currentDrivePower = follower.currentDrivePower;
            this.currentHeadingTarget = follower.currentHeadingTarget;
        }

        @Override
        public String toString() {
            return "DebugPacket for GVF Follower {" +
                    "usingPID=" + usingPID +
                    ", currentClosestTValue=" + currentClosestTValue +
                    ", currentDistanceToEnd=" + currentDistanceToEnd +
                    ", currentClosestPoint=" + currentClosestPoint +
                    ", currentTangentPoint=" + currentTangentPoint +
                    ", currentDrivePower=" + currentDrivePower +
                    ", currentHeadingTarget=" + currentHeadingTarget +
                    '}';
        }
    }

    /**
     * Returns useful information about the current state of the follower.
     * @return DebugPacket
     */
    public DebugPacket getDebugPacket() {
        return new DebugPacket(this);
    }
}
