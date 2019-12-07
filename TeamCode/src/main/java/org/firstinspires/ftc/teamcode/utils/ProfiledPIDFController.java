package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.PIDCoefficients;

public class ProfiledPIDFController {
    private PIDFController pidfController;
    private TrapezoidProfile.State goal = new TrapezoidProfile.State();
    private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();

    private TrapezoidProfile.Constraints constraints;
    private double lastUpdateTimestamp = 0.0;

    public ProfiledPIDFController(PIDCoefficients coeffs, TrapezoidProfile.Constraints constraints) {
        this.pidfController = new PIDFController(coeffs,0.0,0.0,0.0);
        this.constraints = constraints;
    }

    public double calculate(double input) {
        TrapezoidProfile profile = new TrapezoidProfile(constraints, goal, setpoint);
        double currentTimestamp = System.currentTimeMillis() / 10e3;
        double period = currentTimestamp - lastUpdateTimestamp;
        setpoint = profile.calculate(period);
        pidfController.targetPosition = setpoint.position;
        lastUpdateTimestamp = currentTimestamp;
        return  pidfController.update(input);
    }

}
