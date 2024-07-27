package com.roboracers.pathfollower.utils;

import com.roboracers.pathfollower.geometry.Vector2d;
import com.roboracers.pathfollower.planner.CubicBezierCurve;

public class DefaultCurves {

    public static CubicBezierCurve UNIT_S_CURVE = new CubicBezierCurve(
            new Vector2d(0,0), new Vector2d(1,0),
            new Vector2d(0,1), new Vector2d(1,1));
}
