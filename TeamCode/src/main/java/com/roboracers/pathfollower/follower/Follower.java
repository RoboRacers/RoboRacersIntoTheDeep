package com.roboracers.pathfollower.follower;

import com.roboracers.pathfollower.geometry.Pose2d;
import com.roboracers.pathfollower.planner.ParametricPath;

public interface Follower {

    void setPath(ParametricPath parametricPath);
    Pose2d getDriveVelocity();

    Boolean isComplete();
}
