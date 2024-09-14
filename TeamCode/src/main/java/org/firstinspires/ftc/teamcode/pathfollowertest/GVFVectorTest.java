package org.firstinspires.ftc.teamcode.pathfollowertest;


import com.roboracers.topgear.follower.CentripetalGuidedVectorFieldFollower;
import com.roboracers.topgear.geometry.Pose2d;

public class GVFVectorTest {
    public static void main(String[] args) {
        CentripetalGuidedVectorFieldFollower follower = new CentripetalGuidedVectorFieldFollower(1);


        Pose2d driveVel = follower.getDriveVelocity(new Pose2d(35.69, 27.82, Math.toRadians(0)), new Pose2d());

        System.out.println(driveVel);
    }
}
