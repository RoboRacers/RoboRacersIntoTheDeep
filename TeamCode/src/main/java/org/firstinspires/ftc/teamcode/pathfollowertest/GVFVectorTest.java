package org.firstinspires.ftc.teamcode.pathfollowertest;


import com.roboracers.topgear.follower.GuidedVectorFieldFollower;
import com.roboracers.topgear.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.robot.customdrive.TuneableConstants;

public class GVFVectorTest {
    public static void main(String[] args) {
        GuidedVectorFieldFollower follower = new GuidedVectorFieldFollower(TuneableConstants.getParams());


        Pose2d driveVel = follower.getDriveVelocity(new Pose2d(35.69, 27.82, Math.toRadians(0)));

        System.out.println(driveVel);
    }
}
