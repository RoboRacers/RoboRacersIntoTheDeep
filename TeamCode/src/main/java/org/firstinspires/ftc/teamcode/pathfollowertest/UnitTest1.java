package org.firstinspires.ftc.teamcode.pathfollowertest;

import static com.roboracers.pathfollower.utils.DefaultCurves.UNIT_S_CURVE;

import com.roboracers.pathfollower.follower.GuidedVectorFieldFollower;
import com.roboracers.pathfollower.geometry.Pose2d;
import com.roboracers.pathfollower.planner.CubicBezierCurve;

public class UnitTest1 {
    public static void main(String[] args) {
        GuidedVectorFieldFollower follower = new GuidedVectorFieldFollower();

        follower.setPath(UNIT_S_CURVE);

        Pose2d driveVel = follower.getDriveVelocity(new Pose2d(0.51, 0.51, 0));

        System.out.println(driveVel);
    }
}
