package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.localization.Pose2d;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;

public class AutonomousRunner {

    private Robot robot;
    private ElapsedTime globalTimer;

    public AutonomousRunner(Robot robot) {
        this.robot = robot;
        this.globalTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    }

    public void setStrafeRight() {
        followPath(strafeRight(13));
    }

    public void setStrafeLeft() {
        followPath(strafeLeft(13));
    }

    public void setRedBlockPath(double offset) {
        robot.intake.setFoundationServosUp();
        followPath(redStartToBlockTrajectory(offset));

        robot.arm.setArmTargetState(Arm.ControlMode.MACRO_CONTROL, Arm.ArmState.AUTO_PICKUP);
        robot.intake.setControlMode(Intake.ControlMode.MACRO_CONTROL);

        robot.update();

        updateAndBlockFor(3); // give the arm some time to pick up

        followPath(redBlockToFoundationTrajectory(offset));

        robot.arm.setArmTargetState(Arm.ControlMode.MANUAL_CONTROL, Arm.ArmState.HIGH_HOLD);
        robot.intake.setServoState(Intake.ServoState.OPEN);
        robot.intake.setFoundationServosDown();

        updateAndBlockFor(1.25);
        followPath(redFoundationToPark());

        robot.drivetrain.setCurrentEstimatedPose(new Pose2d(10,-80, robot.drivetrain.getEstimatedPose().heading));
        robot.intake.setFoundationServosUp();

        followPath(depotToPark());

    }

    public void setBlueBlockPath(double offset) {
        robot.intake.setFoundationServosUp();
        followPath(blueStartToBlockTrajectory(offset));

        robot.arm.setArmTargetState(Arm.ControlMode.MACRO_CONTROL, Arm.ArmState.AUTO_PICKUP);
        robot.intake.setControlMode(Intake.ControlMode.MACRO_CONTROL);

        robot.update();

        updateAndBlockFor(3); // give the arm some time to pick up

        followPath(redBlockToFoundationTrajectory(offset));

        robot.arm.setArmTargetState(Arm.ControlMode.MANUAL_CONTROL, Arm.ArmState.HIGH_HOLD);
        robot.intake.setServoState(Intake.ServoState.OPEN);
        robot.intake.setFoundationServosDown();

        updateAndBlockFor(1.25);
        followPath(redFoundationToPark());

        robot.drivetrain.setCurrentEstimatedPose(new Pose2d(10,80, robot.drivetrain.getEstimatedPose().heading));
        robot.intake.setFoundationServosUp();

        followPath(redDepotToPark());

    }

    public void setRedFoundationOnly() {
        followPath(redFoundationOnly());
        robot.intake.setFoundationServosDown();

    }

    private void followPath(List<Pose2d> targetPoses) {
        robot.drivetrain.setTargetPoses(targetPoses);
        robot.drivetrain.setState(Drivetrain.State.FOLLOW_PATH);
        updateRobotWhile(() -> robot.drivetrain.getState() != Drivetrain.State.IDLE);
    }
    private List<Pose2d> redStartToBlockTrajectory(double offset) {
        List<Pose2d> targetPoses = new ArrayList<>();
        targetPoses.add(new Pose2d(30,offset,0));
        return targetPoses;
    }

    private List<Pose2d> blueStartToBlockTrajectory(double offset) {
        List<Pose2d> targetPoses = new ArrayList<>();
        targetPoses.add(new Pose2d(30,offset,0));
        return targetPoses;
    }

    private List<Pose2d> redBlockToFoundationTrajectory(double offset) {
        List<Pose2d> targetPoses = new ArrayList<>();
        targetPoses.add(new Pose2d(18,offset,0));
        targetPoses.add(new Pose2d(18,offset,Math.toRadians(90)));
        targetPoses.add(new Pose2d(18, 80, Math.toRadians(90)));
        targetPoses.add(new Pose2d(18, 80, 0));
        targetPoses.add(new Pose2d(34, 80, 0));

        return targetPoses;
    }

    private List<Pose2d> blueBlockToFoundationTrajectory(double offset) {
        List<Pose2d> targetPoses = new ArrayList<>();
        targetPoses.add(new Pose2d(18,offset,0));
        targetPoses.add(new Pose2d(18,offset,Math.toRadians(-90)));
        targetPoses.add(new Pose2d(18, -80, Math.toRadians(-90)));
        targetPoses.add(new Pose2d(18, -80, 0));
        targetPoses.add(new Pose2d(34, -80, 0));

        return targetPoses;
    }

    private List<Pose2d> redFoundationToPark() {
        List<Pose2d> targetPoses = new ArrayList<>();
        targetPoses.add(new Pose2d(34, 80, 0));
        targetPoses.add(new Pose2d(-12, 80, 0));
        targetPoses.add(new Pose2d(-12, 80, Math.toRadians(90)));
        return targetPoses;
    }

    private List<Pose2d> blueFoundationToPark() {
        List<Pose2d> targetPoses = new ArrayList<>();
        targetPoses.add(new Pose2d(34, -80, 0));
        targetPoses.add(new Pose2d(-12, -80, 0));
        targetPoses.add(new Pose2d(-12, -80, Math.toRadians(-90)));
        return targetPoses;
    }

    private List<Pose2d> depotToPark() {
        List<Pose2d> targetPoses = new ArrayList<>();
        targetPoses.add(new Pose2d(10, -40, Math.toRadians(-90)));
        return targetPoses;
    }

    private List<Pose2d> redDepotToPark() {
        List<Pose2d> targetPoses = new ArrayList<>();
        targetPoses.add(new Pose2d(10, 40, Math.toRadians(90)));
        return targetPoses;
    }

    private List<Pose2d> redFoundationOnly() {
        List<Pose2d> targetPoses = new ArrayList<>();
        targetPoses.add(new Pose2d(34,0, 0));
        return targetPoses;
    }

    private List<Pose2d> strafeRight(double amt) {
        List<Pose2d> targetPoses = new ArrayList<>();
        Pose2d currentPos = robot.drivetrain.getEstimatedPose();

        targetPoses.add(new Pose2d(currentPos.x, currentPos.y - amt, currentPos.heading));
        return targetPoses;
    }

    private List<Pose2d> strafeLeft(double amt) {
        List<Pose2d> targetPoses = new ArrayList<>();
        Pose2d currentPos = robot.drivetrain.getEstimatedPose();

        targetPoses.add(new Pose2d(currentPos.x, currentPos.y + amt, currentPos.heading));
        return targetPoses;
    }

    public void updateAndBlockFor(double seconds) {
        globalTimer.reset();
        updateRobotWhile(() -> globalTimer.seconds() < seconds);
    }

    private void updateRobotWhile(BooleanSupplier condition) {
        while (condition.getAsBoolean() && !Thread.currentThread().isInterrupted()) { robot.update(); }
    }



}
