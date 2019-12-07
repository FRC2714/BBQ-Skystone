package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

public class MaxVATester extends LinearOpMode {
    private Robot robot;

    @Override
    public void runOpMode() {
        robot = new Robot(this);
        robot.init();

        double lastPositionX = 0.0;
        double lastVelocity = 0.0;

        double maxVelocity = 0.0;
        double maxAcceleration = 0.0;
        double lastTimeSeconds = System.currentTimeMillis();

        robot.drivetrain.setState(Drivetrain.State.OPEN_LOOP);
        robot.drivetrain.setTargetPowers(1.0,1.0,1.0,1.0);
        waitForStart();

        while(opModeIsActive() && !isStopRequested()) {
            double currentPositionX = robot.drivetrain.getEstimatedPose().x;
            if (currentPositionX > 120) idle();

            double currentTimeSeconds = System.currentTimeMillis() / 10e3;
            double deltaTime = currentTimeSeconds - lastTimeSeconds;

            double currentVelocity = (currentPositionX - lastPositionX) / deltaTime;
            double currentAcceleration = (currentVelocity - lastVelocity) / deltaTime;

            if (currentVelocity > maxVelocity) maxVelocity = currentVelocity;
            if (currentAcceleration > maxAcceleration) maxAcceleration = currentAcceleration;

            telemetry.addData("Max Velocity (inches / second): ", maxVelocity);
            telemetry.addData("Max Acceleration (inches / second^2): ", maxAcceleration);

            telemetry.addData("Max Velocity (meters / second): ", maxVelocity * 0.0254);
            telemetry.addData("Max Acceleration (meters/second^2): ", maxAcceleration * 0.0254);
        }
    }
}
