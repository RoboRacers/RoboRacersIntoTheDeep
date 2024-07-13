package com.roboracers.pathfollower.planner;

import com.roboracers.pathfollower.geometry.Pose2d;
import com.roboracers.pathfollower.geometry.Vector2d;

public interface Path {

    Vector2d getPoint(double t);

}
