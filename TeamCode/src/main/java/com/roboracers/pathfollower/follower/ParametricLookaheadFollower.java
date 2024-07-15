package com.roboracers.pathfollower.follower;

import com.roboracers.pathfollower.CustomMecanumDrive;
import com.roboracers.pathfollower.geometry.PointProjection;
import com.roboracers.pathfollower.geometry.Pose2d;
import com.roboracers.pathfollower.geometry.Vector2d;
import com.roboracers.pathfollower.planner.ParametricPath;

/**
 * A custom following solution that finds the closest point on the
 */
public class ParametricLookaheadFollower implements Follower {

    private ParametricPath parametricPath;
    private CustomMecanumDrive drive;

    private double lastTValue = 0;

    public ParametricLookaheadFollower(CustomMecanumDrive drive) {
        this.drive = drive;
    }

    @Override
    public void setPath(ParametricPath parametricPath) {
        this.parametricPath = parametricPath;
        lastTValue = 0;
    }

    @Override
    public Pose2d getDriveVelocity() {
        // TODO: Scale lookahead based on the arc length of the path
        double lookahead = 0.01;

        // If no path has been set, do not return anything
        if (this.parametricPath == null)
            return null;

        Vector2d currentPosition = drive.getPoseEstimate().vec();

        double closestTValue = PointProjection.projectionGradientDescent(parametricPath, currentPosition, lastTValue);

        Vector2d lookaheadPoint = parametricPath.getPoint(closestTValue + lookahead);

        Vector2d connectingVector = lookaheadPoint.subtract(currentPosition);

        Vector2d normalizedVector = connectingVector.normalize();

        // TODO: Scale velocity based on the distance from the lookahead point
        double velScale = 1.0;

        Vector2d velocityVector = normalizedVector.scalarMultiply(velScale);

        lastTValue = closestTValue;

        return new Pose2d(velocityVector, 0);
    }

    @Override
    public Boolean isComplete() {

        Vector2d endpoint = parametricPath.getPoint(1);

        double delta =  drive.getPoseEstimate().vec().distanceTo(endpoint);

        // TODO: Make this a tunable constant
        double distanceThreshold = 0.1;

        if (delta < distanceThreshold)
            return  true;
        else
            return false;
    }
}
