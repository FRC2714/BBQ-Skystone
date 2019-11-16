package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake implements Subsystem {

    private HardwareMap hardwareMap;

    private DcMotorEx intakeMotor;
    private Servo intakeServo;

    public ServoState servoState = ServoState.OPEN;
    public IntakeState intakeState = IntakeState.IDLE;

    public Intake(HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;

        intakeMotor = this.hardwareMap.get(DcMotorEx.class, "intake_motor");
        intakeServo = this.hardwareMap.get(Servo.class, "intake_servo");

        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void update() {

    }

    public void resetEncoders() {
        intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void runUsingEncoders() {
        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private enum ServoState {
        OPEN,
        CLOSED
    }

    private enum IntakeState {
        IDLE,
        INTAKING,
        EXTAKING,
        HOLDING
    }


}
