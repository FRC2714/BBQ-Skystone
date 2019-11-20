package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import static org.firstinspires.ftc.teamcode.subsystems.Arm.*;

@TeleOp(name = "ArmTest", group = "SubsystemTesting")
public class ArmTest extends LinearOpMode {

    Arm arm;

    @Override
    public void runOpMode() throws InterruptedException {
        arm = getInstance(hardwareMap, telemetry);

        waitForStart();

        while (opModeIsActive()) {
            if(gamepad2.a)
                arm.setArmTargetState(ControlMode.MACRO_CONTROL, ArmState.LOW_HOLD);
            if(gamepad2.b)
                arm.setArmTargetState(ControlMode.MACRO_CONTROL, ArmState.HIGH_HOLD);
            if(gamepad2.x)
                arm.setArmTargetState(ControlMode.MACRO_CONTROL, ArmState.AUTO_PICKUP);
            arm.update();
            telemetry.addData("Target Arm State ", arm.getArmState()[0]);
            telemetry.addData("Current Arm State ", arm.getArmState()[1]);
            telemetry.addData("Control Mode ", arm.controlMode);
            telemetry.addData("Auto Pickup Num " + arm.count, arm.armMacroIterator);
            telemetry.update();
        }
    }
}
