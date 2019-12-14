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

    public DcMotorEx intakeMotor;
    private Servo intakeServo;
    private Servo leftFoundationServo;
    private Servo rightFoundationServo;

    private ElapsedTime elapsedTime;

    public ServoState servoState = ServoState.OPEN;
    public MotorState motorState = MotorState.IDLE;
    public IntakeMacroIterator intakeMacroIterator = IntakeMacroIterator.READY_FOR_STONE;

    private ControlMode controlMode;

    public double userArmPower;

    public static Intake intake;

    public static Intake getInstance(HardwareMap hm, Telemetry tele) {
        if (intake == null) intake = new Intake(hm, tele);
        return intake;
    }

    public Intake(HardwareMap hardwareMap, Telemetry telemetry){
        this.hardwareMap = hardwareMap;
        tele = telemetry;
        intakeMotor = this.hardwareMap.get(DcMotorEx.class, "intake_motor");
        intakeServo = this.hardwareMap.get(Servo.class, "intake_servo");
        leftFoundationServo = this.hardwareMap.get(Servo.class, "foundation_servo_left");
        rightFoundationServo = this.hardwareMap.get(Servo.class, "foundation_servo_right");

        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        controlMode = ControlMode.MANUAL_CONTROL;


    }

    public int count;
    private double startingTime;
    private boolean startedTimer = false;
    @Override
    public void update() {
        switch (controlMode){
            case MACRO_CONTROL:
                switch (intakeMacroIterator){
                    case READY_FOR_STONE:
                        intakeMotor.setPower(-1);
                        intakeServo.setPosition(0.7);
                        if(Arm.getInstance(hardwareMap, tele).getArmMacroIterator() == Arm.ArmMacroIterator.LOW_HOLD)
                            intakeMacroIterator = IntakeMacroIterator.GRAB_STONE;
                        break;
                    case GRAB_STONE:
                        count += -2000000;
                        intakeServo.setPosition(0.4);
                        servoState = ServoState.CLOSED;
                        if(!startedTimer) {
                            startingTime = System.currentTimeMillis();
                            startedTimer = true;
                        }

                        if(System.currentTimeMillis() - startingTime > 0.5e3){
                            intakeMacroIterator = IntakeMacroIterator.HOLDING;
                            tele.addData("BRUH", 0);
                            tele.update();
                        }
                        break;
                    case HOLDING:
                        intakeMotor.setPower(0.05);
                        startedTimer = false;
                        controlMode = ControlMode.MANUAL_CONTROL;
                        intakeMacroIterator = IntakeMacroIterator.READY_FOR_STONE;
                        break;
                }
                break;
            case MANUAL_CONTROL:
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

    public void setControlMode(ControlMode controlMode) {
        this.controlMode = controlMode;
    }

    public void setFoundationServosDown(){
        leftFoundationServo.setPosition(1);
        rightFoundationServo.setPosition(0);
    }

    public void setFoundationServosUp(){
        leftFoundationServo.setPosition(0.5);
        rightFoundationServo.setPosition(0.5);
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