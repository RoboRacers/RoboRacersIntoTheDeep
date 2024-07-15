package com.roboracers.pathfollower.geometry;

import com.roboracers.pathfollower.planner.CubicBezierCurve;
import com.roboracers.pathfollower.planner.ParametricPath;

public class PointProjection {

    // TODO: make these tunable
    private static double EPSILON_SCALING_FACTOR = 100;
    private static double ITERATIONS_SCALING_FACTOR = 10000;

    private static final double LEARNING_RATE_SCALING_FACTOR_GD = 0.01;
    private static final double EPSILON_SCALING_FACTOR_GD = 1e-6;
    private static final double ITERATIONS_SCALING_FACTOR_GD = 1000;

    public static Vector2d findClosestPoint(ParametricPath parametricPath, Vector2d targetPoint) {

        double arclen = parametricPath.getArcLength();
        double epsilon = arclen / EPSILON_SCALING_FACTOR;
        int maxIterations = (int) (arclen * ITERATIONS_SCALING_FACTOR);

        double t = 0.5; // Starting guess for t
        double bestT = t;
        double minDistance = Double.MAX_VALUE;

        for (int i = 0; i < maxIterations; i++) {
            Vector2d P_t = parametricPath.getPoint(t);
            double distance = P_t.distanceTo(targetPoint);
            if (distance < minDistance) {
                minDistance = distance;
                bestT = t;
            }

            // Try small steps in both directions
            double[] steps = {-epsilon, epsilon};
            for (double step : steps) {
                double newT = t + step;
                if (newT >= 0.0 && newT <= 1.0) { // Ensure t is within bounds
                    P_t = parametricPath.getPoint(newT);
                    distance = P_t.distanceTo(targetPoint);
                    if (distance < minDistance) {
                        minDistance = distance;
                        bestT = newT;
                    }
                }
            }
            t = bestT; // Update t for the next iteration
        }

        return parametricPath.getPoint(bestT);

    }

    public static double findClosestTValue(ParametricPath parametricPath, Vector2d targetPoint) {

        double arclen = parametricPath.getArcLength();
        double epsilon = arclen / EPSILON_SCALING_FACTOR;
        int maxIterations = (int) (arclen * ITERATIONS_SCALING_FACTOR);


        double t = 0.5; // Starting guess for t
        double bestT = t;
        double minDistance = Double.MAX_VALUE;

        for (int i = 0; i < maxIterations; i++) {
            Vector2d P_t = parametricPath.getPoint(t);
            double distance = P_t.distanceTo(targetPoint);
            if (distance < minDistance) {
                minDistance = distance;
                bestT = t;
            }

            // Try small steps in both directions
            double[] steps = {-epsilon, epsilon};
            for (double step : steps) {
                double newT = t + step;
                if (newT >= 0.0 && newT <= 1.0) { // Ensure t is within bounds
                    P_t = parametricPath.getPoint(newT);
                    distance = P_t.distanceTo(targetPoint);
                    if (distance < minDistance) {
                        minDistance = distance;
                        bestT = newT;
                    }
                }
            }
            t = bestT; // Update t for the next iteration
        }

        return bestT;

    }

    public static double projectionGradientDescent(ParametricPath parametricPath, Vector2d targetPoint, double guess) {

        // Constants for gradient descent
        double arcLength = 1;
        double learningRate = LEARNING_RATE_SCALING_FACTOR_GD * arcLength;
        double epsilon = EPSILON_SCALING_FACTOR_GD * arcLength;
        int maxIterations = (int) (ITERATIONS_SCALING_FACTOR_GD * arcLength);

        // Initial guess for t
        double t = guess;

        for (int i = 0; i < maxIterations; i++) {
            // Calculate the current point on the path and its derivative
            Vector2d P_t = parametricPath.getPoint(t);
            Vector2d dP_dt = parametricPath.getDerivative(t);

            // Calculate the distance vector from the current point to the target point
            Vector2d distanceVector = P_t.subtract(targetPoint);

            // Calculate the gradient (dot product of distance vector and derivative of the path)
            double gradient = 2 * distanceVector.dot(dP_dt);

            // Update the parameter t using the gradient
            double newT = t - learningRate * gradient;

            // Ensure t is within bounds [0, 1]
            if (newT < 0.0) newT = 0.0;
            if (newT > 1.0) newT = 1.0;

            // Check for convergence
            if (Math.abs(newT - t) < epsilon) break;

            // Update t for the next iteration
            t = newT;
        }

        return t;

    }

    public static void main(String[] args) {
        ParametricPath testParametricPath = new CubicBezierCurve(
                new Vector2d(0,0),
                new Vector2d(0, 1),
                new Vector2d(1,0),
                new Vector2d(1,1)
        );

        Vector2d target = new Vector2d(0.758,0.635);

        double closest = projectionGradientDescent(testParametricPath, target, 0.5);

        System.out.println(
                closest
        );
    }
}
