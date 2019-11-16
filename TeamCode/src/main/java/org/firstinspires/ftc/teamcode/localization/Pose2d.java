package org.firstinspires.ftc.teamcode.localization;

public class Pose2d {
    public double x,y;
    public double heading;

    public Pose2d(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
    }

    public Pose2d(Vector2d vec, double heading) {
        this.x = vec.x;
        this.y = vec.y;
        this.heading = heading;
    }

    public Pose2d() {
        this.x = 0.0;
        this.y = 0.0;
        this.heading = 0.0;
    }


    public Pose2d plus(Pose2d other) { return new Pose2d(x + other.x, y + other.y, heading + other.heading);}
    public Pose2d minus(Pose2d other) { return new Pose2d(x - other.x, y - other.y, heading - other.heading);}
    public Pose2d times(double scalar) { return new Pose2d(x * scalar, y * scalar, heading * scalar); }
    public Pose2d divide(double scalar) { return this.times(1.0 / scalar); }

    public Vector2d vec() { return new Vector2d(x,y); }
    public Vector2d headingVec() { return new Vector2d(Math.cos(heading), Math.sin(heading)); }

}