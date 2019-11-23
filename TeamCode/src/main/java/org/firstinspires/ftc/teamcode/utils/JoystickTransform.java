package org.firstinspires.ftc.teamcode.utils;

import org.firstinspires.ftc.teamcode.localization.Pose2d;

public class JoystickTransform {
    public MODE mode = MODE.LINEAR;
    private double exponent = 3.0;
    public enum MODE {
        LINEAR,
        EXPONENTIAL
    }

    public JoystickTransform() { }
    public Pose2d transform(Pose2d original) {
        double r = original.vec().norm();
        double omega = original.heading;
        double sigOmega = Math.signum(omega);
        omega = Math.abs(omega);

        switch (mode) {
            case LINEAR:
                break;
            case EXPONENTIAL:
                r = Math.pow(r, exponent);
                omega = Math.pow(omega, exponent);
                break;
        }

        Pose2d command = new Pose2d(original.vec().times(r / original.vec().norm()), omega * sigOmega);
        if (r == 0) command = new Pose2d(0, 0, omega * sigOmega);
        return command;
    }

    public void setMode(MODE mode) {
        this.mode = mode;
    }
}
