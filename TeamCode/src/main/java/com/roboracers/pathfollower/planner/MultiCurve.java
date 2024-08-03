package com.roboracers.pathfollower.planner;

import com.roboracers.pathfollower.geometry.Vector2d;

import java.util.List;

public class MultiCurve implements ParametricPath{
    private List<ParametricPath> curves;

    public MultiCurve(List<ParametricPath> curves) {
        this.curves = curves;
    }

    @Override
    public Vector2d getPoint(double t) {
        int n = curves.size();
        double scaledT = t * n; // Scale t to the number of curves
        int curveIndex = (int) Math.min(scaledT, n - 1); // Find the current curve index
        double localT = scaledT - curveIndex; // Calculate the local t for the curve

        return curves.get(curveIndex).getPoint(localT);
    }

    @Override
    public Vector2d getDerivative(double t) {
        int n = curves.size();
        double scaledT = t * n; // Scale t to the number of curves
        int curveIndex = (int) Math.min(scaledT, n - 1); // Find the current curve index
        double localT = scaledT - curveIndex; // Calculate the local t for the curve

        return curves.get(curveIndex).getDerivative(localT);
    }

    @Override
    public double getArcLength() {
        double arcLength = 0;

        for (ParametricPath curve : curves) {
            arcLength += curve.getArcLength();
        }
        return arcLength;
    }

}
