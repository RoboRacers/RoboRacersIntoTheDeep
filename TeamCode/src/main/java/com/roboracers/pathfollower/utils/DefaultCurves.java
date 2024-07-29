package com.roboracers.pathfollower.utils;

import com.roboracers.pathfollower.geometry.Vector2d;
import com.roboracers.pathfollower.planner.CubicBezierCurve;

public class DefaultCurves {

    public static CubicBezierCurve UNIT_S_CURVE = new CubicBezierCurve(
            new Vector2d(0,0), new Vector2d(1,0),
            new Vector2d(0,1), new Vector2d(1,1));

    public static CubicBezierCurve FIELD_S_CURVE = new CubicBezierCurve(
            new Vector2d(0,0), new Vector2d(72,0),
            new Vector2d(0,72), new Vector2d(72,72));

    public static CubicBezierCurve HALF_FIELD_S_CURVE = new CubicBezierCurve(
            new Vector2d(0,0), new Vector2d(48,0),
            new Vector2d(0,48), new Vector2d(48,48));
}
