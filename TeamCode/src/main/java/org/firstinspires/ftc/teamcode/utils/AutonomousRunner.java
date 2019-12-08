package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.localization.Pose2d;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

import java.util.ArrayList;
import java.util.List;

public class AutonomousRunner {

    private Robot robot;
    public AutonomousRunner(Robot robot) {
        this.robot = robot;
    }

    public void setBlueBlockPath() {
        followPath(blueStartToBlockTrajectory());

        robot.arm.setArmTargetState(Arm.ControlMode.MACRO_CONTROL, Arm.ArmState.AUTO_PICKUP);
        robot.intake.setControlMode(Intake.ControlMode.MACRO_CONTROL);

        robot.update();

        updateAndBlockFor(3); // give the arm some time to pick up

        followPath(blueBlockToFoundationTrajectory());

        robot.intake.setServoState(Intake.ServoState.OPEN);
    }

    private void followPath(List<Pose2d> targetPoses) {
        robot.drivetrain.setTargetPoses(targetPoses);
        robot.drivetrain.setState(Drivetrain.State.FOLLOW_PATH);
        while (robot.drivetrain.getState() != Drivetrain.State.IDLE) { robot.update(); robot.telemetry.update(); }
    }

    private List<Pose2d> blueStartToBlockTrajectory() {
        List<Pose2d> targetPoses = new ArrayList<>();
        targetPoses.add(new Pose2d(30,0,0));
        return targetPoses;
    }

    private List<Pose2d> blueBlockToFoundationTrajectory() {
        List<Pose2d> targetPoses = new ArrayList<>();
        targetPoses.add(new Pose2d(24,0,0));
        targetPoses.add(new Pose2d(18,0,0));
        targetPoses.add(new Pose2d(24,0,Math.toRadians(-90)));
        targetPoses.add(new Pose2d(24, -80, Math.toRadians(-90)));
        targetPoses.add(new Pose2d(24, -80, 0));
        targetPoses.add(new Pose2d(33, -80, 0));

        return targetPoses;
    }

    private void updateAndBlockFor(double seconds) {
        ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        while(timer.seconds() < seconds) {
            robot.update();
        }
    }

}
