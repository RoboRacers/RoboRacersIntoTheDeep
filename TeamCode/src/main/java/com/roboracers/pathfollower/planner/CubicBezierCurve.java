package com.roboracers.pathfollower.planner;

import com.roboracers.pathfollower.geometry.Vector2d;

import java.util.ArrayList;

public class CubicBezierCurve implements ParametricPath {
    private final Vector2d controlPoint1;
    private final Vector2d controlPoint2;
    private final Vector2d controlPoint3;
    private final Vector2d controlPoint4;

    private final double arcLength;

    // TODO: Make this tunable
    public static int ARCLEN_ESTIMATION_SEGMENTS = 100;

    public CubicBezierCurve(Vector2d controlPoint1, Vector2d controlPoint2, Vector2d controlPoint3, Vector2d controlPoint4) {
        this.controlPoint1 = controlPoint1;
        this.controlPoint2 = controlPoint2;
        this.controlPoint3 = controlPoint3;
        this.controlPoint4 = controlPoint4;

        this.arcLength = this.bezierArcLength(ARCLEN_ESTIMATION_SEGMENTS);
    }

    public CubicBezierCurve(ArrayList<Vector2d> controlPoints) {
        this.controlPoint1 = controlPoints.get(0);
        this.controlPoint2 = controlPoints.get(1);
        this.controlPoint3 = controlPoints.get(2);
        this.controlPoint4 = controlPoints.get(3);

        this.arcLength = this.bezierArcLength(ARCLEN_ESTIMATION_SEGMENTS);
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

    @Override
    public Vector2d getDerivative(double t) {
        double u = 1 - t;

        Vector2d term1 = controlPoint2.subtract(controlPoint1).multiply(3 * u * u); // 3(1-t)^2 * (P1 - P0)
        Vector2d term2 = controlPoint3.subtract(controlPoint2).multiply(6 * u * t); // 6(1-t)t * (P2 - P1)
        Vector2d term3 = controlPoint4.subtract(controlPoint3).multiply(3 * t * t); // 3t^2 * (P3 - P2)

        return term1.add(term2).add(term3);

    }

    private double bezierArcLength(int numSegments) {
        double totalLength = 0.0;

        for (int i = 0; i < numSegments; i++) {
            double t0 = (double) i / numSegments;
            double t1 = (double) (i + 1) / numSegments;

            Vector2d p0 = getPoint(t0);
            Vector2d p1 = getPoint(t1);

            totalLength += p0.distanceTo(p1);
        }

        return totalLength;
    }

    public double getArcLength() {
        return arcLength;
    }

    public static void main(String[] args) {
        CubicBezierCurve testPath = new CubicBezierCurve(
                new Vector2d(0,0),
                new Vector2d(0, 1),
                new Vector2d(0,1),
                new Vector2d(0,2)
        );

        double len = testPath.bezierArcLength(100);

        System.out.println(len);
    }

}
