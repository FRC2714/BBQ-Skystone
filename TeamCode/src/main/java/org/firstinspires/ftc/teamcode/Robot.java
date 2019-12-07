package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Subsystem;

import java.util.ArrayList;
import java.util.List;

public class Robot {
    public Telemetry telemetry;
    private HardwareMap hw;

    public Drivetrain drivetrain;
    public Arm arm;
    public Intake intake;

    List<Subsystem> subsystems;
    public Robot(OpMode opMode) {
        this.telemetry = opMode.telemetry;
        this.hw = opMode.hardwareMap;
        subsystems = new ArrayList<>();
    }

    public void init() {
        arm = Arm.getInstance(hw, telemetry);
        intake = Intake.getInstance(hw, telemetry);
        drivetrain = new Drivetrain(hw, telemetry, arm);

       subsystems.add(drivetrain);
       subsystems.add(arm);
       subsystems.add(intake);
    }

    public void run() {
        for (Subsystem s : subsystems)
            s.update();
    }
}
