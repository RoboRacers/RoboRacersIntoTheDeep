package com.roboracers.pathfollower.follower;

import com.roboracers.pathfollower.geometry.Pose2d;
import com.roboracers.pathfollower.planner.ParametricPath;

public interface Follower {

    void setPath(ParametricPath parametricPath);

    Pose2d getDriveVelocity(Pose2d currentPosition);

    Boolean isComplete(Pose2d currentPosition);
}
