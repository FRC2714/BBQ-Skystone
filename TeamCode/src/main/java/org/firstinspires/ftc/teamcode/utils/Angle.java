package org.firstinspires.ftc.teamcode.utils;

public class Angle {
    private static final double TAU = Math.PI * 2;

    private static double norm(double angle) {
        double newAngle = angle % TAU;

        newAngle = (newAngle + TAU) % TAU;

        if (newAngle > Math.PI)
            newAngle -= TAU;

        return newAngle;
    }

    public static double normDelta(double angle) {
        double newAngleDelta = norm(angle);
        if (newAngleDelta > Math.PI) {
            newAngleDelta -= TAU;
        }

        return newAngleDelta;
    }

}
