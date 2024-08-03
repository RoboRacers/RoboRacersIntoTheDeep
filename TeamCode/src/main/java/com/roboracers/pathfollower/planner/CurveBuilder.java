package com.roboracers.pathfollower.planner;

import com.roboracers.pathfollower.geometry.Vector2d;

import java.util.ArrayList;
import java.util.List;

public class CurveBuilder {

    public static CubicBezierCurve buildCurve(Vector2d start, Vector2d end, double startTangent, double endTangent) {
        double k = 0.5;

        double D = start.distanceTo(end);

        Vector2d tangent1 = new Vector2d(Math.cos(startTangent), Math.sin(startTangent));
        Vector2d tangent2 = new Vector2d(Math.cos(endTangent), Math.sin(endTangent));

        double a = k * D;
        double b = -k * D;

        Vector2d control1 = start.add(tangent1.multiply(a));
        Vector2d control2 = end.subtract(tangent2.multiply(b));

        return new CubicBezierCurve(start, control1, control2, end);
    }

    public static CubicBezierCurve buildCurve(Vector2d start, Vector2d end, double startTangent, double endTangent, double curvature) {
        double k = curvature;

        double D = start.distanceTo(end);

        Vector2d tangent1 = new Vector2d(Math.cos(startTangent), Math.sin(startTangent));
        Vector2d tangent2 = new Vector2d(Math.cos(endTangent), Math.sin(endTangent));

        double a = k * D;
        double b = -k * D;

        Vector2d control1 = start.add(tangent1.multiply(a));
        Vector2d control2 = end.subtract(tangent2.multiply(b));

        return new CubicBezierCurve(start, control1, control2, end);
    }

    public static CurveList buildCurveSequence() {
        return new CurveList();
    }

    public static class CurveList {
        List<ParametricPath> curves;

        public CurveList() {
            curves = new ArrayList<>();
        }

        public CurveList addCurve(Vector2d start, Vector2d end, double startTangent, double endTangent) {
            curves.add(buildCurve(start, end, startTangent, endTangent));
            return this;
        }

        public CurveList addCurve(Vector2d start, Vector2d end, double startTangent, double endTangent, double curvature) {
            curves.add(buildCurve(start, end, startTangent, endTangent, curvature));
            return this;
        }

        /**
         * Merges multiple bezier curves into a single parametric path. The curves are concatenated in the order they are added.
         * @return the parametric path, with a domain from 0 to 1.
         */
        public  ParametricPath build() {
            return new MultiCurve(curves);
        }
    }
}
