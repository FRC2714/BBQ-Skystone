package org.firstinspires.ftc.teamcode.localization;

public class Vector2d {
    public double x, y;

    public Vector2d(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public Vector2d() {
        this.x = 0;
        this.y = 0;
    }

    public double norm() { return Math.sqrt(x * x + y * y); }
    public double angle() { return Math.atan2(y, x); }
    public Vector2d add(Vector2d other) { return new Vector2d(x + other.x, y + other.y); }
    public Vector2d minus(Vector2d other) { return new Vector2d(x - other.x, y - other.y); }
    public Vector2d times(double scalar) { return new Vector2d(x * scalar, y * scalar); }
    public Vector2d divide(double scalar) { return new Vector2d(x / scalar, y  / scalar); }

    public Vector2d rotated(double angle) {
        double newX = x * Math.cos(angle) - y * Math.sin(angle);
        double newY = x * Math.sin(angle) + y * Math.cos(angle);
        return new Vector2d(newX, newY);
    }

}