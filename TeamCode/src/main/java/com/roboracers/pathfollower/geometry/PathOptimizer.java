package com.roboracers.pathfollower.geometry;
import com.roboracers.pathfollower.planner.CubicBezierCurve;
import com.roboracers.pathfollower.planner.Path;

import org.apache.commons.math3.analysis.MultivariateFunction;
import org.apache.commons.math3.analysis.MultivariateVectorFunction;
import org.apache.commons.math3.optim.MaxEval;
import org.apache.commons.math3.optim.PointValuePair;
import org.apache.commons.math3.optim.SimpleBounds;
import org.apache.commons.math3.optim.SimpleValueChecker;
import org.apache.commons.math3.optim.nonlinear.scalar.GoalType;
import org.apache.commons.math3.optim.nonlinear.scalar.MultivariateOptimizer;
import org.apache.commons.math3.optim.nonlinear.scalar.noderiv.BOBYQAOptimizer;
import org.apache.commons.math3.optim.nonlinear.scalar.noderiv.CMAESOptimizer;
import org.apache.commons.math3.optim.nonlinear.scalar.noderiv.SimplexOptimizer;
import org.apache.commons.math3.optim.nonlinear.scalar.noderiv.PowellOptimizer;
import org.apache.commons.math3.optim.nonlinear.scalar.noderiv.BOBYQAOptimizer;
import org.apache.commons.math3.optim.nonlinear.scalar.noderiv.PowellOptimizer;
import org.apache.commons.math3.optim.nonlinear.scalar.noderiv.SimplexOptimizer;
import org.apache.commons.math3.optim.nonlinear.scalar.gradient.NonLinearConjugateGradientOptimizer;
import org.apache.commons.math3.optim.nonlinear.scalar.gradient.NonLinearConjugateGradientOptimizer.Formula;
import org.apache.commons.math3.optim.nonlinear.scalar.gradient.NonLinearConjugateGradientOptimizer.IdentityPreconditioner;
import org.apache.commons.math3.optim.nonlinear.scalar.ObjectiveFunction;
import org.apache.commons.math3.optim.InitialGuess;
import org.apache.commons.math3.optim.PointValuePair;

public class PathOptimizer {
    public static Vector2d findClosestPoint(Path path, Vector2d targetPoint) {
        // Define the parametric path P(t) for the given path
        MultivariateVectorFunction parametricPathFunction = new MultivariateVectorFunction() {
            @Override
            public double[] value(double[] t) {
                return new double[] {
                        path.getPoint(t[0]).getX(),
                        path.getPoint(t[0]).getY()
                };
            }
        };

        // Define the distance function to minimize
        MultivariateFunction distance = new MultivariateFunction() {
            @Override
            public double value(double[] t) {
                double[] P_t = parametricPathFunction.value(t);
                double dx = P_t[0] - targetPoint.getX();
                double dy = P_t[1] - targetPoint.getY();
                return Math.sqrt(dx * dx + dy * dy);
            }
        };

        // Optimization settings
        MultivariateOptimizer optimizer = new NonLinearConjugateGradientOptimizer(
                Formula.POLAK_RIBIERE, new SimpleValueChecker(1e-9, 1e-9)
        );
        double[] startPoint = {0.5}; // Starting guess for t
        double[] lowerBound = {0.0}; // Lower bound for t (assuming t is in range [0, 1])
        double[] upperBound = {1.0}; // Upper bound for t

        // Optimize to find the t that minimizes the distance
        PointValuePair result = optimizer.optimize(
                new MaxEval(10000),
                new InitialGuess(startPoint),
                new SimpleBounds(lowerBound, upperBound),
                GoalType.MINIMIZE,
                new ObjectiveFunction(distance)
        );

        double t_min = result.getPoint()[0];
        return path.getPoint(t_min);
    }

    public static void main(String[] args) {
        Path testPath = new CubicBezierCurve(
                new Vector2d(0,0),
                new Vector2d(0, 1),
                new Vector2d(1,0),
                new Vector2d(1,1)
        );

        Vector2d target = new Vector2d(0.5, 0.7);

        Vector2d closest = findClosestPoint(testPath, target);

        System.out.println(
                closest
        );
    }
}
