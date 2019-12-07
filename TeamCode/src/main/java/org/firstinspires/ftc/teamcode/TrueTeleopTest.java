package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.localization.Pose2d;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.utils.JoystickTransform;

@TeleOp(name = "TrueTeleopTest")
public class TrueTeleopTest extends LinearOpMode {
    Robot robot;
    JoystickTransform transform;
    private Arm arm;
    private Intake intake;

    @Override
    public void runOpMode() {
        arm = Arm.getInstance(hardwareMap, telemetry);
        intake = Intake.getInstance(hardwareMap, telemetry);

        robot = new Robot(this);
        robot.init();
        transform = new JoystickTransform();

        robot.drivetrain.setState(Drivetrain.State.TELEOP);

        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {

            if(gamepad2.a)
                arm.setArmTargetState(Arm.ControlMode.MACRO_CONTROL, Arm.ArmState.LOW_HOLD);
            if(gamepad2.b)
                arm.setArmTargetState(Arm.ControlMode.MACRO_CONTROL, Arm.ArmState.HIGH_HOLD);
            if(gamepad2.x) {
                arm.setArmTargetState(Arm.ControlMode.MACRO_CONTROL, Arm.ArmState.AUTO_PICKUP);
                intake.setControlMode(Intake.ControlMode.MACRO_CONTROL);
            }

            arm.setArmPower(gamepad2.left_stick_y);

            if(gamepad2.left_bumper)
                intake.setUserArmPower(1);
            else if(Math.abs(gamepad2.left_trigger) > 0.3)
                intake.setUserArmPower(-1);
            else
                intake.setUserArmPower(0);

            if(gamepad2.right_bumper)
                intake.setServoState(Intake.ServoState.OPEN);
            if(Math.abs(gamepad2.right_trigger) > 0.3)
                intake.setServoState(Intake.ServoState.CLOSED);

            telemetry.addData("Servo State: ", intake.servoState);
            telemetry.addData("User Arm Power: ", intake.userArmPower);
            telemetry.addData("Counting manual Loop ", intake.count);
            telemetry.addData("Right Trigger", gamepad2.right_trigger);
            telemetry.update();

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
