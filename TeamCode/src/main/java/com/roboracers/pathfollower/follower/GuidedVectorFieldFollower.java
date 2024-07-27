package com.roboracers.pathfollower.follower;

import com.roboracers.pathfollower.geometry.PointProjection;
import com.roboracers.pathfollower.geometry.Pose2d;
import com.roboracers.pathfollower.geometry.Vector2d;
import com.roboracers.pathfollower.planner.ParametricPath;

/**
 *
 */
public class GuidedVectorFieldFollower implements Follower {

    /**
     * Current parametrically defined path that is being follower.
     */
    private ParametricPath parametricPath;
    private double tangentDistance;
    /**
     * Value of the previous closest point, for faster point projection
     * (finding the closest point on the path from the robot's position)
     */
    private double lastTValue = 0;

    public GuidedVectorFieldFollower(double tangentDistance) {
        this.tangentDistance = tangentDistance;
    }

    /**
     * Set the current path to be followed.
     * @param parametricPath
     */
    @Override
    public void setPath(ParametricPath parametricPath) {
        this.parametricPath = parametricPath;
        lastTValue = 0;
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

        Vector2d currentPoint = currentPosition.vec();

        double closestTValue = PointProjection.projectionGradientDescent(parametricPath, currentPoint, lastTValue);

        Vector2d tangentPoint = parametricPath.getPoint(closestTValue).add(
                parametricPath.getDerivative(closestTValue).normalize().multiply(tangentDistance));

        Vector2d connectingVector = tangentPoint.subtract(currentPoint);

        Vector2d normalizedVector = connectingVector.normalize();

        // TODO: Scale velocity based on the distance from the tangentDistance point
        double velScale = 1.0;

        Vector2d velocityVector = normalizedVector.scalarMultiply(velScale);

        lastTValue = closestTValue;

        return new Pose2d(velocityVector, 0);
    }

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
}
