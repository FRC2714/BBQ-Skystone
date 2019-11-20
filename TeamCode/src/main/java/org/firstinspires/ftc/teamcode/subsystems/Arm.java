package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Arm implements Subsystem {

    private static Arm arm;
    private HardwareMap hm;

    private DcMotorEx armMotor;
    private AnalogInput armPotentiometer;

    private ArmState currentArmState;
    private ArmState targetArmState;
    public ControlMode controlMode;
    public ArmMacroIterator armMacroIterator;

    boolean macroRunOnce = true;

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

    public int count = 0;
    @Override
    public void update() {

        switch (controlMode){
            case MACRO_CONTROL:
                if (!armMotor.isBusy() || macroRunOnce){

                    switch (targetArmState) {
                        case START:
                            goToPosition(-10, 0.5);
                            macroRunOnce = false;
                            break;
                        case HIGH_HOLD:
                            goToPosition(-350, 1);
                            macroRunOnce = false;
                            break;
                        case LOW_HOLD:
                            goToPosition(-1100, 1);
                            macroRunOnce = false;
                            break;
                        case STONE_PICKUP:
                            goToPosition(-1470, 1);
                            macroRunOnce = false;
                            break;
                        case AUTO_PICKUP:
                            switch (armMacroIterator){
                                case STONE_PICKUP:
                                    count++;
                                    goToPosition(-1470, 1);
                                    if(isArmAtTarget()) {
                                        count += 1000;
                                        armMacroIterator = ArmMacroIterator.LOW_HOLD;
                                    }
                                    break;
                                case LOW_HOLD:
                                    count += 1000000;
                                    goToPosition(-1100, 1);
                                    if(isArmAtTarget()) {
                                        macroRunOnce = false;
                                    }
                                    break;
                            }
                            break;
                    }
                }
                if(isArmAtTarget() && !macroRunOnce) {
                    controlMode = ControlMode.MANUAL_CONTROL;
                    armMacroIterator = ArmMacroIterator.STONE_PICKUP;
                    currentArmState = targetArmState;
                    macroRunOnce = true;
                }
                break;
            case MANUAL_CONTROL:
                armMotor.setPower(0);
                break;
        }
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

    public void setArmTargetState(ControlMode controlMode, ArmState armState){
        targetArmState = armState;
        this.controlMode = controlMode;
        tele.addData("Setting Arm Target State", 0);
        tele.update();
    }

    public enum ArmState {
        START,
        HIGH_HOLD,
        LOW_HOLD,
        STONE_PICKUP,
        AUTO_PICKUP
    }

    public enum ControlMode{
        MANUAL_CONTROL,
        MACRO_CONTROL
    }

    private enum ArmMacroIterator{
        STONE_PICKUP,
        LOW_HOLD
    }

}
