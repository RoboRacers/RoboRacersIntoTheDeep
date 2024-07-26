package com.roboracers.pathfollower.geometry;

public class Vector2d {
    private final double x;
    private final double y;

    public Vector2d(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public Vector2d add(Vector2d v) {
        return new Vector2d(this.x + v.x, this.y + v.y);
    }

    public Vector2d subtract(Vector2d other) {
        return new Vector2d(this.x - other.x, this.y - other.y);
    }

    public Vector2d multiply(double scalar) {
        return new Vector2d(this.x * scalar, this.y * scalar);
    }

    public double dot(Vector2d v) {
        return this.x * v.x + this.y * v.y;
    }

    public double length() {
        return Math.sqrt(x * x + y * y);
    }

    public Vector2d normalize() {
        double length = length();
        if (length == 0) {
            throw new ArithmeticException("Cannot normalize a zero-length vector");
        }
        return new Vector2d(x / length, y / length);
    }

    public Vector2d scalarMultiply(double scalar) {
        return new Vector2d(x * scalar, y * scalar);
    }

    public double distanceTo(Vector2d other) {
        double dx = this.x - other.x;
        double dy = this.y - other.y;
        return Math.sqrt(dx * dx + dy * dy);
    }
}