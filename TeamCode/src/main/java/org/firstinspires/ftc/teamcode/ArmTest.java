package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

import static org.firstinspires.ftc.teamcode.subsystems.Arm.*;

@TeleOp(name = "ArmTest", group = "SubsystemTesting")
public class ArmTest extends LinearOpMode {

    private Arm arm;
    private Intake intake;

    @Override
    public void runOpMode() throws InterruptedException {
        arm = Arm.getInstance(hardwareMap, telemetry);
        intake = Intake.getInstance(hardwareMap, telemetry);

        waitForStart();

        while (opModeIsActive()) {
            if(gamepad2.a)
                arm.setArmTargetState(ControlMode.MACRO_CONTROL, ArmState.LOW_HOLD);
            if(gamepad2.b)
                arm.setArmTargetState(ControlMode.MACRO_CONTROL, ArmState.HIGH_HOLD);
            if(gamepad2.x)
                arm.setArmTargetState(ControlMode.MACRO_CONTROL, ArmState.AUTO_PICKUP);

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

            arm.update();
            intake.update();
//            telemetry.addData("Target Arm State ", arm.getArmState()[0]);
//            telemetry.addData("Current Arm State ", arm.getArmState()[1]);
//            telemetry.addData("Control Mode ", arm.controlMode);
//            telemetry.addData("Auto Pickup Num " + arm.count, arm.armMacroIterator);

            telemetry.addData("Servo State: ", intake.servoState);
            telemetry.addData("User Arm Power: ", intake.userArmPower);
            telemetry.addData("Counting manual Loop ", intake.count);
            telemetry.addData("Right Trigger", gamepad2.right_trigger);
            telemetry.update();
        }
    }
}
