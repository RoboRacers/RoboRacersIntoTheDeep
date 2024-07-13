package com.roboracers.pathfollower.follower;

import com.roboracers.pathfollower.geometry.Pose2d;
import com.roboracers.pathfollower.planner.Path;

public class CustomFollower implements Follower {

    private Path path;

    @Override
    public void setPath(Path path) {
        this.path = path;
    }

    @Override
    public Pose2d getDriveVelocity() {
        return null;
    }

    @Override
    public Boolean isComplete() {
        return null;
    }
}
