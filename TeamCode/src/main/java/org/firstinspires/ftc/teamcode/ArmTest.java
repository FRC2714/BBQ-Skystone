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
            arm.update();
            telemetry.update();
        }
    }
}
