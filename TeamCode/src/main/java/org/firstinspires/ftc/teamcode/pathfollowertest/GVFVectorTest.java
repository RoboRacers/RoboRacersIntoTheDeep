package org.firstinspires.ftc.teamcode.pathfollowertest;

import static com.roboracers.pathfollower.utils.DefaultCurves.FIELD_S_CURVE;
import static com.roboracers.pathfollower.utils.DefaultCurves.UNIT_S_CURVE;

import com.roboracers.pathfollower.follower.GuidedVectorFieldFollower;
import com.roboracers.pathfollower.geometry.Pose2d;

public class GVFVectorTest {
    public static void main(String[] args) {
        GuidedVectorFieldFollower follower = new GuidedVectorFieldFollower(0.5);

        follower.setPath(FIELD_S_CURVE);

        Pose2d driveVel = follower.getDriveVelocity(new Pose2d(35.69, 27.82, Math.toRadians(0)));

        System.out.println(driveVel);
    }
}
