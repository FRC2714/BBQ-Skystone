package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.localization.Pose2d;
import org.firstinspires.ftc.teamcode.utils.PIDFController;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;

import java.util.ArrayList;
import java.util.List;

public class Drivetrain implements Subsystem {
    private ExpansionHubEx expansionHub;
    private RevBulkData bulkData;
    private HardwareMap hwMap;

    private ExpansionHubMotor fl, fr, bl, br;
    private List<ExpansionHubMotor> motors;

    // PIDF Controllers
    private PIDFController translationalController;
    private PIDFController lateralController;
    private PIDFController headingController;

    // targets
    private Pose2d targetVelocity;

    // current values
    private State currentState = State.TELEOP;
    private Pose2d currentVelocity;

    enum State {
        TELEOP,
        OPEN_LOOP,
        CLOSED_LOOP
    }

    public Drivetrain(HardwareMap map) {
        this.hwMap = map;
        this.expansionHub = hwMap.get(ExpansionHubEx.class, "Expansion Hub 2");

        fl = (ExpansionHubMotor) hwMap.dcMotor.get("front_left");
        fr = (ExpansionHubMotor) hwMap.dcMotor.get("front_right");
        bl = (ExpansionHubMotor) hwMap.dcMotor.get("back_left");
        br = (ExpansionHubMotor) hwMap.dcMotor.get("back_right");
        motors.add(fl);
        motors.add(bl);
        motors.add(br);
        motors.add(fr);

    }

    @Override
    public void update() {
        bulkData = expansionHub.getBulkInputData();
        switch(currentState) {
            case CLOSED_LOOP:
        }
    }

}
