package org.firstinspires.ftc.teamcode.localization;

import java.util.ArrayList;
import java.util.List;

public class MecanumKinematics {

    public static Pose2d wheelToRobotVelocity(List<Double> wheelVelocities, double trackWidth) {
        // front left, rear left, rear right, front right
        return new Pose2d(
                wheelVelocities.stream().mapToDouble(a -> a).sum(),
                wheelVelocities.get(1) + wheelVelocities.get(3) - wheelVelocities.get(0) - wheelVelocities.get(2),
                (wheelVelocities.get(2) + wheelVelocities.get(3) - wheelVelocities.get(0)  - wheelVelocities.get(1)) / trackWidth
        ).times(0.25);
    }

    public static List<Double> robotToWheelVelocites(Pose2d robotVel, double trackWidth) {
        List<Double> wheelVelos = new ArrayList<>();
        wheelVelos.add(robotVel.x - robotVel.y - trackWidth * robotVel.heading);
        wheelVelos.add(robotVel.x + robotVel.y - trackWidth * robotVel.heading);
        wheelVelos.add(robotVel.x - robotVel.y + trackWidth * robotVel.heading);
        wheelVelos.add(robotVel.x + robotVel.y + trackWidth * robotVel.heading);
        return wheelVelos;
    }
}