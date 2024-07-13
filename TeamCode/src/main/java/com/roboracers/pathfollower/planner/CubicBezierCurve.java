package com.roboracers.pathfollower.planner;

import com.roboracers.pathfollower.geometry.Vector2d;
public class CubicBezierCurve implements Path {
    private Vector2d controlPoint1;
    private Vector2d controlPoint2;
    private Vector2d controlPoint3;
    private Vector2d controlPoint4;

    public CubicBezierCurve(Vector2d controlPoint1, Vector2d controlPoint2, Vector2d controlPoint3, Vector2d controlPoint4) {
        this.controlPoint1 = controlPoint1;
        this.controlPoint2 = controlPoint2;
        this.controlPoint3 = controlPoint3;
        this.controlPoint4 = controlPoint4;
    }

    public Vector2d getPoint(double t) {

        double oneMinusT = 1.0 - t;

        double x = Math.pow(oneMinusT, 3) * controlPoint1.getX() +
                3 * Math.pow(oneMinusT, 2) * t * controlPoint2.getX() +
                3 * oneMinusT * Math.pow(t, 2) * controlPoint3.getX() +
                Math.pow(t, 3) * controlPoint4.getX();

        double y = Math.pow(oneMinusT, 3) * controlPoint1.getY() +
                3 * Math.pow(oneMinusT, 2) * t * controlPoint2.getY() +
                3 * oneMinusT * Math.pow(t, 2) * controlPoint3.getY() +
                Math.pow(t, 3) * controlPoint4.getY();

        return new Vector2d(x, y);
    }

}
