package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake implements Subsystem {

    private HardwareMap hardwareMap;

    private DcMotorEx intakeMotor;
    private Servo intakeServo;

    public ServoState servoState = ServoState.OPEN;
    public MotorState motorState = MotorState.IDLE;

    private ControlMode controlMode;

    public double userArmPower;

    private static Intake intake;

    public static Intake getInstance(HardwareMap hm, Telemetry tele) {
        if (intake == null) intake = new Intake(hm, tele);
        return intake;
    }

    public Intake(HardwareMap hardwareMap, Telemetry telemetry){
        this.hardwareMap = hardwareMap;

        intakeMotor = this.hardwareMap.get(DcMotorEx.class, "intake_motor");
        intakeServo = this.hardwareMap.get(Servo.class, "intake_servo");

        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        controlMode = ControlMode.MANUAL_CONTROL;
    }

    public int count;
    @Override
    public void update() {
        switch (controlMode){
            case MACRO_CONTROL:

                break;
            case MANUAL_CONTROL:
                count++;
                intakeMotor.setPower(userArmPower);
                switch (servoState){
                    case OPEN:
                        intakeServo.setPosition(0.7);
                        break;
                    case CLOSED:
                        intakeServo.setPosition(0.4);
                        break;
                }
                break;
        }
    }

    public void setUserArmPower(double userArmPower){
        this.userArmPower = userArmPower;
    }

    public void setServoState(ServoState servoState){
        this.servoState = servoState;
    }

    public void resetEncoders() {
        intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void runUsingEncoders() {
        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public enum ServoState {
        OPEN,
        CLOSED
    }

    private enum MotorState {
        IDLE,
        INTAKING,
        EXTAKING,
        HOLDING
    }

    public enum ControlMode{
        MANUAL_CONTROL,
        MACRO_CONTROL
    }

}
