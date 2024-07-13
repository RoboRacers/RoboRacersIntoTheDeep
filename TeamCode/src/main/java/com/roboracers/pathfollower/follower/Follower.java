package com.roboracers.pathfollower.follower;

import com.roboracers.pathfollower.geometry.Pose2d;
import com.roboracers.pathfollower.planner.Path;

public interface Follower {

    void setPath(Path path);
    Pose2d getDriveVelocity();

    Boolean isComplete();
}
