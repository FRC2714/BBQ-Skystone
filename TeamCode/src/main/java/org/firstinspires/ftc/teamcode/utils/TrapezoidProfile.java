package org.firstinspires.ftc.teamcode.utils;

public class TrapezoidProfile {
    // The direction of the profile, either 1 for forwards or -1 for inverted
    private int direction;
    private Constraints constraints;

    private State start;
    private State goal;

    private double endAccel;
    private double endFullSpeed;
    private double endDeccel;

    public static class Constraints {
        public double maxVelocity;
        public double maxAcceleration;

        public Constraints() {}

        public Constraints(double maxVelocity, double maxAcceleration) {
            this.maxVelocity = maxVelocity;
            this.maxAcceleration = maxAcceleration;
        }
    }

    public static class State {
        public double position;
        public double velocity;

        public State() {}

        public State(double position, double velocity) {
            this.position = position;
            this.velocity = velocity;
        }

        @Override
        public boolean equals(Object other) {
            if (other instanceof State) {
                State rhs = (State) other;
                return this.position == rhs.position && this.velocity == rhs.velocity;
            } else {
                return false;
            }
        }
    }

    public TrapezoidProfile(Constraints constraints, State start, State goal) {
        direction = goal.position < start.position ? 1 : -1;
        this.constraints = constraints;
        this.start = direct(start);
        this.goal = direct(goal);

        if (start.velocity > constraints.maxVelocity) {
            start.velocity = constraints.maxVelocity;
        }

        // Deal with a possibly truncated motion profile (with nonzero initial or
        // final velocity) by calculating the parameters as if the profile began and
        // ended at zero velocity
        double cutoffBegin = start.velocity / constraints.maxAcceleration;
        double cutoffDistBegin = cutoffBegin * cutoffBegin * constraints.maxAcceleration / 2.0;

        double cutoffEnd = goal.velocity / constraints.maxAcceleration;
        double cutoffDistEnd = cutoffEnd * cutoffEnd * constraints.maxAcceleration / 2.0;

        // Now we can calculate the parameters as if it was a full trapezoid instead
        // of a truncated one

        double fullTrapezoidDist = cutoffDistBegin + (goal.position - start.position)
                + cutoffDistEnd;
        double accelerationTime = constraints.maxVelocity / constraints.maxAcceleration;

        double fullSpeedDist = fullTrapezoidDist - accelerationTime * accelerationTime
                * constraints.maxAcceleration;

        // Handle the case where the profile never reaches full speed
        if (fullSpeedDist < 0) {
            accelerationTime = Math.sqrt(fullTrapezoidDist / constraints.maxAcceleration);
            fullSpeedDist = 0;
        }

        endAccel = accelerationTime - cutoffBegin;
        endFullSpeed = endAccel + fullSpeedDist / constraints.maxVelocity;
        endDeccel = endFullSpeed + accelerationTime - cutoffEnd;
    }

    /**
     * Calculate the correct position and velocity for the profile at a time t
     * where the beginning of the profile was at time t = 0.
     *
     * @param t The time since the beginning of the profile.
     */
    public State calculate(double t) {
        State result = start;

        if (t < endAccel) {
            result.velocity += t * constraints.maxAcceleration;
            result.position += (start.velocity + t * constraints.maxAcceleration / 2.0) * t;
        } else if (t < endFullSpeed) {
            result.velocity = constraints.maxVelocity;
            result.position += (start.velocity + endAccel * constraints.maxAcceleration
                    / 2.0) * endAccel + constraints.maxVelocity * (t - endAccel);
        } else if (t <= endDeccel) {
            result.velocity = goal.velocity + (endDeccel - t) * constraints.maxAcceleration;
            double timeLeft = endDeccel - t;
            result.position = goal.position - (goal.velocity + timeLeft
                    * constraints.maxAcceleration / 2.0) * timeLeft;
        } else {
            result = goal;
        }

        return direct(result);
    }

    /**
     * Returns the total time the profile takes to reach the goal.
     */
    public double totalTime() {
        return endDeccel;
    }

    /**
     * Returns true if the profile has reached the goal.
     *
     * <p>The profile has reached the goal if the time since the profile started
     * has exceeded the profile's total time.
     *
     * @param t The time since the beginning of the profile.
     */
    @SuppressWarnings("ParameterName")
    public boolean isFinished(double t) {
        return t >= totalTime();
    }

    private State direct(State in) {
        State result = new State(in.position, in.velocity);
        result.position = result.position * direction;
        result.velocity = result.velocity * direction;
        return result;
    }
}
