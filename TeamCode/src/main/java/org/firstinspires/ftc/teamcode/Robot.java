package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Subsystem;

import java.util.ArrayList;
import java.util.List;

public class Robot {
    private Telemetry telemetry;
    private HardwareMap hw;

    public Drivetrain drivetrain;

    List<Subsystem> subsystems;
    public Robot(OpMode opMode) {
        this.telemetry = opMode.telemetry;
        this.hw = opMode.hardwareMap;
        subsystems = new ArrayList<>();
    }

    public void init() {
       drivetrain = new Drivetrain(hw, telemetry);
       subsystems.add(drivetrain);
    }

    public void run() {
        for (Subsystem s : subsystems)
            s.update();
    }
}