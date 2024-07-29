package org.firstinspires.ftc.teamcode.pathfollowertest;

import static com.roboracers.pathfollower.utils.DefaultCurves.UNIT_S_CURVE;

import com.roboracers.pathfollower.follower.GuidedVectorFieldFollower;
import com.roboracers.pathfollower.geometry.Pose2d;
import com.roboracers.pathfollower.geometry.Vector2d;

public class fieldCentricToRobotCentricTest {
    public static void main(String[] args) {
        Vector2d unitVec = new Vector2d(1,1);

        System.out.println(unitVec.fieldToRobotCentric(Math.toRadians(90)).toString());
    }
}
