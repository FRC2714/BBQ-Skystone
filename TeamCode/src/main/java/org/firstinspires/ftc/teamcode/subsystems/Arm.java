package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Arm implements Subsystem {

    public static Arm arm;
    private HardwareMap hm;

    private DcMotorEx armMotor;
    private AnalogInput armPotentiometer;

    private ArmState currentArmState;
    private ArmState targetArmState;
    public ControlMode controlMode;
    public ArmMacroIterator armMacroIterator;

    private double userArmPower;

    private boolean macroRunAgain = true;

    private Telemetry tele;

    public static Arm getInstance(HardwareMap hm, Telemetry tele) {
        if (arm == null) arm = new Arm(hm, tele);
        return arm;
    }

    public Arm(HardwareMap hm, Telemetry tele) {
        this.hm = hm;
        this.tele = tele;
        armMotor = this.hm.get(DcMotorEx.class, "arm_motor");

        armPotentiometer = this.hm.get(AnalogInput.class, "arm_potentiometer");

        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        targetArmState = ArmState.START;
        controlMode = ControlMode.MANUAL_CONTROL;
        armMacroIterator = ArmMacroIterator.STONE_PICKUP;
    }

//    public void resetArmAfterAuton(){
//       targetArmState = ArmState.ARM_RESET;
//    }

    public int count = 0;
    public boolean isMoving = true;
    @Override
    public void update() {

        switch (controlMode){
            case MACRO_CONTROL:

                if (!armMotor.isBusy() || macroRunAgain){
                    switch (targetArmState) {
                        case START:
                            goToPosition(-10, 0.5);
                            macroRunAgain = false;
                            break;
                        case HIGH_HOLD:
                            goToPosition(-350, 1);
                            macroRunAgain = false;
                            break;
                        case LOW_HOLD:
                            goToPosition(-1250, 1);
                            macroRunAgain = false;
                            break;
                        case STONE_PICKUP:
                            goToPosition(-1470, 1);
                            macroRunAgain = false;
                            break;
                        case AUTO_PICKUP:
                            isMoving = true;
                            switch (armMacroIterator){
                                case STONE_PICKUP:
                                    count++;
                                    goToPosition(-1550, 1);
                                    if(isArmAtTarget()) {
                                        // tele.addData("BRUH MOMENT = ", 222222222);
                                        armMacroIterator = ArmMacroIterator.LOW_HOLD;
                                    }
                                    break;
                                case LOW_HOLD:
                                    if(Intake.getInstance(hm,tele).intakeMacroIterator == Intake.IntakeMacroIterator.HOLDING) {
                                        goToPosition(-1250, 1);
                                        if (isArmAtTarget()) {
                                            macroRunAgain = false;
                                        }

                                        if(!armMotor.isBusy())
                                            isMoving = false;
                                    }
                                    break;
                            }
                            break;
                        case ARM_RESET:
                            goToPosition(350, 1);
                            if(isArmAtTarget()) {
                                resetEncoders();
                                targetArmState = ArmState.START;
                            }
                            break;

                    }
                }
                if(isArmAtTarget() && !macroRunAgain) {
                    controlMode = ControlMode.MANUAL_CONTROL;
                    armMacroIterator = ArmMacroIterator.STONE_PICKUP;
                    currentArmState = targetArmState;
                    macroRunAgain = true;
                }
                break;
            case MANUAL_CONTROL:
                armMotor.setPower(userArmPower);
                break;
        }
    }

    public boolean isMacroRunAgain() {
        return macroRunAgain;
    }

    private void resetEncoders() {
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private void setRunUsingEncoders() {
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void setRunToPosition() {
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    private boolean isArmAtTarget(){
        return (Math.abs(armMotor.getCurrentPosition() - armMotor.getTargetPosition()) < 30);
    }

    private void goToPosition(int targetPosition , double power){
        armMotor.setTargetPosition(targetPosition);
        setRunToPosition();
        armMotor.setPower(power);
    }

    public ArmState[] getArmState(){
        return new ArmState[]{targetArmState, currentArmState};
    }

    public ArmMacroIterator getArmMacroIterator(){
        return armMacroIterator;
    }

    public void setArmTargetState(ControlMode controlMode, ArmState armState){
        targetArmState = armState;
        this.controlMode = controlMode;
        tele.addData("Setting Arm Target State", 0);
        tele.update();
    }

    public void setArmPower(double power){
        userArmPower = power;
    }

    public boolean moving() {
        return armMotor.isBusy() && Intake.getInstance(hm, tele).intakeMotor.isBusy();
    }

    public enum ArmState {
        START,
        HIGH_HOLD,
        LOW_HOLD,
        STONE_PICKUP,
        AUTO_PICKUP,
        ARM_RESET
    }

    public enum ControlMode{
        MANUAL_CONTROL,
        MACRO_CONTROL
    }

    public enum ArmMacroIterator{
        STONE_PICKUP,
        LOW_HOLD
    }

    public void resetArm() {
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}