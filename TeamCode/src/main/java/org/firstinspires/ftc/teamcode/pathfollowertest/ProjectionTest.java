package org.firstinspires.ftc.teamcode.pathfollowertest;

import static com.roboracers.pathfollower.utils.DefaultCurves.FIELD_S_CURVE;

import com.roboracers.pathfollower.follower.GuidedVectorFieldFollower;
import com.roboracers.pathfollower.geometry.PointProjection;
import com.roboracers.pathfollower.geometry.Pose2d;
import com.roboracers.pathfollower.geometry.Vector2d;

public class ProjectionTest {
    public static void main(String[] args) {
        double t = PointProjection.projectionBinarySearch(FIELD_S_CURVE, new Vector2d(40, 0), 10);

        System.out.println(t);
        System.out.println(FIELD_S_CURVE.getPoint(t).toString());

    }
}
