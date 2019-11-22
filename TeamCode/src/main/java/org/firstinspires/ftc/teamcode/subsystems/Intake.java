package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.concurrent.TimeUnit;

public class Intake implements Subsystem {

    private HardwareMap hardwareMap;
    private Telemetry tele;

    private DcMotorEx intakeMotor;
    private Servo intakeServo;

    private ElapsedTime elapsedTime;

    public ServoState servoState = ServoState.OPEN;
    public MotorState motorState = MotorState.IDLE;
    public IntakeMacroIterator intakeMacroIterator = IntakeMacroIterator.READY_FOR_STONE;

    private ControlMode controlMode;

    public double userArmPower;

    private static Intake intake;

    public static Intake getInstance(HardwareMap hm, Telemetry tele) {
        if (intake == null) intake = new Intake(hm, tele);
        return intake;
    }

    public Intake(HardwareMap hardwareMap, Telemetry telemetry){
        this.hardwareMap = hardwareMap;
        tele = telemetry;
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

                switch (intakeMacroIterator){
                    case READY_FOR_STONE:
                        intakeMotor.setPower(1);
                        intakeServo.setPosition(0.7);
                        if(Arm.getInstance(hardwareMap, tele).getArmMacroIterator() == Arm.ArmMacroIterator.LOW_HOLD)
                            intakeMacroIterator = IntakeMacroIterator.GRAB_STONE;
                        break;
                    case GRAB_STONE:
                        intakeServo.setPosition(0.4);
                        if(runIntakeMotorForTime(0.4)){
                            intakeMacroIterator = IntakeMacroIterator.HOLDING;
                        }
                        break;
                    case HOLDING:
                        intakeMotor.setPower(0.05);
                        controlMode = ControlMode.MANUAL_CONTROL;
                        intakeMacroIterator = IntakeMacroIterator.READY_FOR_STONE;
                        break;
                }
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

    public IntakeMacroIterator getIntakeMacroIterator() {
        return intakeMacroIterator;
    }

    public void resetEncoders() {
        intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void runUsingEncoders() {
        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private boolean startedTimer = false;
    public boolean runIntakeMotorForTime(double seconds){
        if(!startedTimer) {
            elapsedTime = new ElapsedTime(0);
            startedTimer = true;
        }
        if(elapsedTime.time(TimeUnit.SECONDS) > seconds)
            return true;
        else
            intakeMotor.setPower(1);
        return false;
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

    public enum IntakeMacroIterator{
        READY_FOR_STONE,
        GRAB_STONE,
        HOLDING,
    }

}
