package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.localization.Pose2d;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

import java.util.ArrayList;
import java.util.List;

public class AutonmousScheduler {
    private Robot robot;
    public AutonmousScheduler(Robot robot) {
        this.robot = robot;
    }

    public void setBlueBlockPath() {
        List<Pose2d> targetPoses = new ArrayList<>();
        targetPoses.add(new Pose2d(30,0,0));
        robot.drivetrain.setTargetPoses(targetPoses);
        robot.drivetrain.setState(Drivetrain.State.FOLLOW_PATH);
        while (robot.drivetrain.getState() != Drivetrain.State.IDLE) { robot.run(); robot.telemetry.update(); }
        robot.arm.setArmTargetState(Arm.ControlMode.MACRO_CONTROL, Arm.ArmState.AUTO_PICKUP);
        robot.intake.setControlMode(Intake.ControlMode.MACRO_CONTROL);
        int count = 0;
        robot.run();
        ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

        while(timer.seconds() < 3) {
            robot.run();
            robot.telemetry.addData("Arm States = " + robot.arm.getArmMacroIterator(), 0);
            robot.telemetry.update();
        }

        targetPoses = new ArrayList<>();
        targetPoses.add(new Pose2d(24,0,0));
        targetPoses.add(new Pose2d(18,0,0));
        targetPoses.add(new Pose2d(24,0,Math.toRadians(-90)));
        targetPoses.add(new Pose2d(24, -80, Math.toRadians(-90)));
        targetPoses.add(new Pose2d(24, -80, 0));
        targetPoses.add(new Pose2d(33, -80, 0));
        robot.drivetrain.setTargetPoses(targetPoses);
        robot.drivetrain.setState(Drivetrain.State.FOLLOW_PATH);
        while (robot.drivetrain.getState() != Drivetrain.State.IDLE) { robot.run(); robot.telemetry.update(); }
        robot.intake.setServoState(Intake.ServoState.OPEN);
    }
}
