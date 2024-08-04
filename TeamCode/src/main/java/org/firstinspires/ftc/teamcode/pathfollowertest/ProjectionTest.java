package org.firstinspires.ftc.teamcode.pathfollowertest;

import static com.roboracers.topgear.utils.DefaultCurves.FIELD_S_CURVE;

import com.roboracers.topgear.geometry.PointProjection;
import com.roboracers.topgear.geometry.Vector2d;

public class ProjectionTest {
    public static void main(String[] args) {
        double t = PointProjection.projectionBinarySearch(FIELD_S_CURVE, new Vector2d(40, 0), 10);

        System.out.println(t);
        System.out.println(FIELD_S_CURVE.getPoint(t).toString());

    }
}
