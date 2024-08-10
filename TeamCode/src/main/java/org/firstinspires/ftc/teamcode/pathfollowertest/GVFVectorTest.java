package org.firstinspires.ftc.teamcode.pathfollowertest;

import static com.roboracers.topgear.utils.DefaultCurves.FIELD_S_CURVE;

import com.roboracers.topgear.follower.GuidedVectorFieldFollower;
import com.roboracers.topgear.geometry.Pose2d;

public class GVFVectorTest {
    public static void main(String[] args) {
        GuidedVectorFieldFollower follower = new GuidedVectorFieldFollower(0.5);

        follower.setPath(FIELD_S_CURVE);

        Pose2d driveVel = follower.getDriveVelocity(new Pose2d(35.69, 27.82, Math.toRadians(0)));

        System.out.println(driveVel);
    }
}
