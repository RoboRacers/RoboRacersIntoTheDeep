package com.roboracers.pathfollower.geometry;

import static java.lang.Math.cos;
import static java.lang.Math.sin;

/**
 * Class for representing 2D robot poses (x, y, and heading) and their derivatives.
 * Clone of Roadrunner's Pose2d class
 */
public class Pose2d {
    private final double x;
    private final double y;
    private final double heading;

    public Pose2d() {
        this(0.0, 0.0, 0.0);
    }

    public Pose2d(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getHeading() {
        return heading;
    }

    public Pose2d(Vector2d pos, double heading) {
        this(pos.getX(), pos.getY(), heading);
    }

    public Vector2d vec() {
        return new Vector2d(x, y);
    }

    public Vector2d headingVec() {
        return new Vector2d(cos(heading), sin(heading));
    }

    public Pose2d plus(Pose2d other) {
        return new Pose2d(x + other.x, y + other.y, heading + other.heading);
    }

    public Pose2d minus(Pose2d other) {
        return new Pose2d(x - other.x, y - other.y, heading - other.heading);
    }

    public Pose2d times(double scalar) {
        return new Pose2d(scalar * x, scalar * y, scalar * heading);
    }

    public Pose2d div(double scalar) {
        return new Pose2d(x / scalar, y / scalar, heading / scalar);
    }

    public Pose2d unaryMinus() {
        return new Pose2d(-x, -y, -heading);
    }

    @Override
    public String toString() {
        return String.format("(%.3f, %.3f, %.3f°)", x, y, Math.toDegrees(heading));
    }
}


