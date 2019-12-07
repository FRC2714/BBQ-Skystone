package org.firstinspires.ftc.teamcode.localization;

import org.firstinspires.ftc.teamcode.utils.Angle;

import java.util.ArrayList;
import java.util.List;

public class Kinematics {
    public static Pose2d fieldToRobotPoseVelocity(Pose2d fieldPose, Pose2d fieldPoseVelocity) {
        return new Pose2d(fieldPoseVelocity.vec().rotated(-fieldPose.heading), fieldPoseVelocity.heading);
    }

    public static List<Double> calculateMotorFeedforward(List<Double> velocities, List<Double> acclerations, double kV, double kA, double kStatic) {
        List<Double> motorPowers = new ArrayList<>();
        for (double v : velocities) {
            motorPowers.add(kV * v);
        }
        return motorPowers;
    }

    /**
     * Performs a relative odometry update. This assumes the robot is moving at a constant velocity over the interval
     * @param fieldPose represents the robots field pose
     * @
     */

    public static Pose2d odometryUpdate(Pose2d fieldPose, Pose2d robotPoseDelta) {
        Pose2d fieldPoseDelta;

        if (Math.abs(robotPoseDelta.heading) > 1e-6) {
            double finalHeading = fieldPose.heading + robotPoseDelta.heading;
            double cosTerm = Math.cos(finalHeading) - Math.cos(fieldPose.heading);
            double sinTerm = Math.sin(finalHeading) - Math.sin(fieldPose.heading);

            fieldPoseDelta = new Pose2d(
                    (robotPoseDelta.x * sinTerm + robotPoseDelta.y * cosTerm) / robotPoseDelta.heading,
                    (-robotPoseDelta.x * cosTerm + robotPoseDelta.y * sinTerm) / robotPoseDelta.heading,
                    robotPoseDelta.heading
            );
        } else {
            fieldPoseDelta = new Pose2d(
                    robotPoseDelta.vec().rotated(fieldPose.heading + robotPoseDelta.heading / 2),
                    robotPoseDelta.heading
            );
        }
        return fieldPose.plus(fieldPoseDelta);
    }

    public static Pose2d calculatePoseError(Pose2d targetFieldPose, Pose2d currentFieldPose) {
        return new Pose2d(
                targetFieldPose.minus(currentFieldPose).vec().rotated(-currentFieldPose.heading),
                Angle.normDelta(targetFieldPose.heading - currentFieldPose.heading)
        );
    }


}