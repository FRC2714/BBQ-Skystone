package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Arm implements Subsystem {

    private static Arm arm;
    private HardwareMap hm;

    private DcMotorEx armMotor;
    private AnalogInput armPotentiometer;

    public ArmState armState = ArmState.START;


    public static Arm getInstance(HardwareMap hm) {
        if (arm == null) arm = new Arm(hm);
        return arm;
    }

    public Arm(HardwareMap hm) {
        this.hm = hm;

        armMotor = this.hm.get(DcMotorEx.class, "arm_motor");

        armPotentiometer = this.hm.get(AnalogInput.class, "arm_potentiometer");

        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armState = ArmState.START;
    }

    @Override
    public void update() {

    }

    public void resetEncoders() {
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void runUsingEncoders() {
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void runToPosition() {
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    private enum ArmState {
        START,
        LOW_HOLD,
        HIGH_HOLD,
        STONE_PICKUP;
    }

}
