package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class PIDFController {
    private double errorSum = 0.0;
    private double lastUpdateTimestamp = Double.NaN;

    private boolean inputBounded = false;
    private double minInput = 0.0;
    private double maxInput = 0.0;

    private boolean outputBounded = false;
    private double minOutput = 0.0;
    private double maxOutput = 0.0;

    // Controller setpoint
    private double targetPosition = 0.0;
    private double lastError = 0.0;

    private PIDCoefficients pid;
    private double kV, kA, kStatic = 0.0;

    public PIDFController(PIDCoefficients pid, double kV, double kA, double kStatic) { // TODO: add kF

    }

    public void setInputBounds(double min, double max) {
        if (min < max) {
            inputBounded = true;
            minInput = min;
            maxInput = max;
        }
    }

    public void setOutputBounds(double min, double max) {
        if (min < max) {
            outputBounded = true;
            minOutput = min;
            maxOutput = max;
        }
    }

    private double getError(double pos) {
        double error = targetPosition - pos;
        if (inputBounded) {
            double inputRange = maxInput - minInput;
            while (Math.abs(error) > inputRange / 2.0) {
                error -= Math.signum(error) * inputRange;
            }
        }
        return error;
    }

    public double update(double pos, double velocity, double acceleration) {
        return internalUpdate(pos,velocity,acceleration);
    }
    public double update(double pos) {
        return internalUpdate(pos,0.0,0.0);
    }

    private double internalUpdate(double pos, double velocity, double acceleration) {
        double currentTimestamp = System.currentTimeMillis() * 10e3;
        double error = getError(pos);
        if (Double.isNaN(currentTimestamp)) {
            lastError = error;
            lastUpdateTimestamp = currentTimestamp;
            return 0.0;
        } else {
            double dt = currentTimestamp - lastUpdateTimestamp;
            errorSum += 0.5 * (error + lastError) * dt;
            double errorDeriv = (error - lastError) / dt;

            lastError = error;
            lastUpdateTimestamp = currentTimestamp;

            double baseOutput = pid.p * error + pid.i * errorSum + pid.d * (errorDeriv - velocity)
                    + kV * velocity + kA * acceleration; // add kf soon

            double output;
            if (Math.abs(baseOutput - 0.0) < 0.001) {
                output = 0.0;
            } else {
                output = baseOutput + Math.signum(baseOutput) * kStatic;
            }

            if (outputBounded)
                return Math.max(minOutput, Math.min(output, maxOutput));
            else
                return output;
        }
    }

    public void reset() {
        errorSum = 0.0;
        lastError = 0.0;
        lastUpdateTimestamp = Double.NaN;
    }

}
