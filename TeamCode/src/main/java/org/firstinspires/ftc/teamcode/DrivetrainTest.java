package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.localization.Pose2d;
import org.firstinspires.ftc.teamcode.utils.JoystickTransform;

@TeleOp(name="DrivetrainTest")
public class DrivetrainTest extends LinearOpMode {
    Robot robot;
    JoystickTransform transform;
    @Override
    public void runOpMode() {
        robot = new Robot(this);
        robot.init();
        transform = new JoystickTransform();
        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            robot.run();
            Pose2d v = transform.transform(new Pose2d(-gamepad1.left_stick_x, -gamepad1.left_stick_y, -gamepad1.right_stick_x));
            robot.drivetrain.setControllerVelocity(v);
            telemetry.addData("x: ", -gamepad1.left_stick_y);
            telemetry.addData("y: ", -gamepad1.left_stick_x);
            telemetry.addData("heading: ", -gamepad1.right_stick_x);

            telemetry.update();
        }
    }
}
