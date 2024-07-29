package com.roboracers.pathfollower.geometry;

import com.roboracers.pathfollower.planner.CubicBezierCurve;
import com.roboracers.pathfollower.planner.ParametricPath;

public class PointProjection {

    private static final double INITIAL_LEARNING_RATE = 0.01;
    private static final double EPSILON = 1e-4;
    private static final int MAX_ITERATIONS = 10000;
    private static final int NUM_GUESSES = 20; // Number of evenly-spread guesses

    public static double projectionGradientDescent(ParametricPath parametricPath, Vector2d targetPoint, double guessRange) {
        double bestT = 0.0;
        double bestDistance = Double.MAX_VALUE;

        // Spread guesses evenly across the range [0, 1]
        for (int i = 0; i < NUM_GUESSES; i++) {
            double initialGuess = i / (double) (NUM_GUESSES - 1) * guessRange; // Spread guesses evenly

            double t = initialGuess;
            double learningRate = INITIAL_LEARNING_RATE;

            for (int iter = 0; iter < MAX_ITERATIONS; iter++) {
                // Calculate the current point on the path and its derivative
                Vector2d P_t = parametricPath.getPoint(t);
                Vector2d dP_dt = parametricPath.getDerivative(t);

                // Calculate the distance vector from the current point to the target point
                Vector2d distanceVector = P_t.subtract(targetPoint);

                // Calculate the gradient
                double gradient = 2 * distanceVector.dot(dP_dt);

                // Update the parameter t
                double newT = t - learningRate * gradient;

                // Ensure t is within bounds [0, 1]
                newT = Math.max(0.0, Math.min(1.0, newT));

                // Check for convergence
                if (Math.abs(newT - t) < EPSILON) {
                    // Calculate the distance from the target point
                    double currentDistance = P_t.distanceTo(targetPoint);

                    // Update the best result if needed
                    if (currentDistance < bestDistance) {
                        bestDistance = currentDistance;
                        bestT = newT;
                    }
                    break;
                }

                // Update t for the next iteration
                t = newT;

                // Optionally adjust learning rate (e.g., decay)
                // learningRate *= 0.99;
            }
        }

        return bestT;
    }

    /**
     * Projection method that uses binary search to find the t-value of a point
     * on a path that is closest to the given target point.
     * Credit: Function taken from Pedro Pathing's implementation
     * @param path Path to search on
     * @param targetPoint The target point to minimize distance to
     * @param searchStepLimit The amount of iterations for the binary search
     * @return t-value of the closest point on the path
     */
    public static double projectionBinarySearch(ParametricPath path, Vector2d targetPoint, int searchStepLimit) {
        double lower = 0;
        double upper = 1;
        Pose2d returnPoint;

        // we don't need to calculate the midpoint, so we start off at the 1/4 and 3/4 point
        for (int i = 0; i < searchStepLimit; i++) {
            if ( targetPoint.distanceTo(path.getPoint(lower + 0.25 * (upper-lower))) > targetPoint.distanceTo(path.getPoint(lower + 0.75 * (upper-lower)))) {
                lower += (upper-lower)/2.0;
            } else {
                upper -= (upper-lower)/2.0;
            }
        }

        double closestPointTValue = lower + 0.5 * (upper-lower);

        return closestPointTValue;

    }

    public static void main(String[] args) {
        ParametricPath testParametricPath = new CubicBezierCurve(
                new Vector2d(0,0),
                new Vector2d(0, 1),
                new Vector2d(1,0),
                new Vector2d(1,1)
        );

        Vector2d target = new Vector2d(0.78,0.5);

        double closest = projectionGradientDescent(testParametricPath, target, 0);

        System.out.println(
                testParametricPath.getPoint(closest)
        );
    }
}
