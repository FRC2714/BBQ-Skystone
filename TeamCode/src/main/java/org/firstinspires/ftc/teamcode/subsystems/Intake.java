package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import javax.security.auth.Subject;

public class Intake implements Subsystem {

    private DcMotorEx intakeMotor;
    private Servo intakeServo;

    private HardwareMap hardwareMap;

    public ServoState servoState = ServoState.OPEN;
    public IntakeState intakeState = IntakeState.IDLE;

    public Intake(HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;

        intakeMotor = this.hardwareMap.get(DcMotorEx.class, "intake_motor");
        intakeServo = this.hardwareMap.get(Servo.class, "intake_servo");

        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void resetEncoders() {
        intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void runUsingEncoders() {
        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void update() {

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
