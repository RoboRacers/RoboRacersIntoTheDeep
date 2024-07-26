package com.roboracers.pathfollower.localization;

import static org.apache.commons.math3.util.FastMath.cos;
import static org.apache.commons.math3.util.FastMath.sin;

import com.roboracers.pathfollower.geometry.Pose2d;
import com.roboracers.pathfollower.geometry.Vector2d;
import com.roboracers.pathfollower.utils.AngleUtils;

import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.DecompositionSolver;
import org.apache.commons.math3.linear.LUDecomposition;
import org.apache.commons.math3.linear.MatrixUtils;

import java.util.ArrayList;
import java.util.List;

public abstract class ThreeTrackingWheelLocalizer implements Localizer {
    private Pose2d poseEstimate = new Pose2d();
    private List<Double> lastWheelPositions = new ArrayList<>();
    private DecompositionSolver forwardSolver;

    @Override
    public Pose2d getPoseEstimate() {
        return poseEstimate;
    }

    @Override
    public void setPoseEstimate(Pose2d value) {
        lastWheelPositions = new ArrayList<>();
        poseEstimate = value;
    }

    public Pose2d poseVelocity = null;

    public ThreeTrackingWheelLocalizer(List<Pose2d> wheelPoses) {
        if (wheelPoses.size() != 3) {
            throw new IllegalArgumentException("3 wheel positions must be provided");
        }

        Array2DRowRealMatrix inverseMatrix = new Array2DRowRealMatrix(3, 3);
        for (int i = 0; i < 3; i++) {
            Vector2d orientationVector = wheelPoses.get(i).headingVec();
            Vector2d positionVector = wheelPoses.get(i).vec();
            inverseMatrix.setEntry(i, 0, orientationVector.getX());
            inverseMatrix.setEntry(i, 1, orientationVector.getY());
            inverseMatrix.setEntry(i, 2, positionVector.getX() * orientationVector.getY() - positionVector.getY() * orientationVector.getX());
        }

        forwardSolver = new LUDecomposition(inverseMatrix).getSolver();

        if (!forwardSolver.isNonSingular()) {
            throw new IllegalArgumentException("The specified configuration cannot support full localization");
        }
    }

    private Pose2d calculatePoseDelta(List<Double> wheelDeltas) {
        double[][] rawPoseDelta = forwardSolver.solve(MatrixUtils.createRealMatrix(new double[][]{wheelDeltas.stream().mapToDouble(Double::doubleValue).toArray()}).transpose()).getData();
        return new Pose2d(rawPoseDelta[0][0], rawPoseDelta[1][0], rawPoseDelta[2][0]);
    }

    @Override
    public void update() {
        List<Double> wheelPositions = getWheelPositions();
        if (!lastWheelPositions.isEmpty()) {
            List<Double> wheelDeltas = new ArrayList<>();
            for (int i = 0; i < wheelPositions.size(); i++) {
                wheelDeltas.add(wheelPositions.get(i) - lastWheelPositions.get(i));
            }
            Pose2d robotPoseDelta = calculatePoseDelta(wheelDeltas);
            poseEstimate = relativeOdometryUpdate(poseEstimate, robotPoseDelta);
        }

        List<Double> wheelVelocities = getWheelVelocities();
        if (wheelVelocities != null) {
            poseVelocity = calculatePoseDelta(wheelVelocities);
        }

        lastWheelPositions = wheelPositions;
    }

    /**
     * Returns the positions of the tracking wheels in the desired distance units (not encoder counts!)
     */
    protected abstract List<Double> getWheelPositions();

    /**
     * Returns the velocities of the tracking wheels in the desired distance units (not encoder counts!)
     */
    public List<Double> getWheelVelocities() {
        return null;
    }

    /**
     * Static Methods
    * */


    /**
     * Performs a relative odometry update. Note: this assumes that the robot moves with constant velocity over the
     * measurement interval.
     */
    public static Pose2d relativeOdometryUpdate(Pose2d fieldPose, Pose2d robotPoseDelta) {
        double dtheta = robotPoseDelta.getHeading();
        double sineTerm, cosTerm;

        if (epsilonEquals(dtheta, 0.0)) {
            sineTerm = 1.0 - dtheta * dtheta / 6.0;
            cosTerm = dtheta / 2.0;
        } else {
            sineTerm = sin(dtheta) / dtheta;
            cosTerm = (1 - cos(dtheta)) / dtheta;
        }

        Vector2d fieldPositionDelta = new Vector2d(
                sineTerm * robotPoseDelta.getX() - cosTerm * robotPoseDelta.getY(),
                cosTerm * robotPoseDelta.getX() + sineTerm * robotPoseDelta.getY()
        );

        Pose2d fieldPoseDelta = new Pose2d(fieldPositionDelta.rotated(fieldPose.getHeading()), robotPoseDelta.getHeading());

        return new Pose2d(
                fieldPose.getX() + fieldPoseDelta.getX(),
                fieldPose.getY() + fieldPoseDelta.getY(),
                AngleUtils.norm(fieldPose.getHeading() + fieldPoseDelta.getHeading())
        );
    }

    private static boolean epsilonEquals(double a, double b) {
        return Math.abs(a - b) < 1e-9; // Define your epsilon value
    }

}
