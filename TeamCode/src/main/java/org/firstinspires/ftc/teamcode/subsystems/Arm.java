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
    private ControlMode controlMode;

    boolean macroRunOnce = true;

    int armTargetPosition;
    double armTargetVelocity;
    double intakeTargetVelocity = 0;

    Telemetry tele;

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
    }

    @Override
    public void update() {

        switch (controlMode){
            case MACRO_CONTROL:
                if (!armMotor.isBusy() || macroRunOnce){

                    switch (targetArmState) {
                        case START:
                            goToPosition(-10, 0.5);
                            break;
                        case HIGH_HOLD:
                            goToPosition(-350, 1);
                            break;
                        case LOW_HOLD:
                            goToPosition(-1100, 1);
                            break;
                        case STONE_PICKUP:
                            goToPosition(-1470, 1);
                            break;
                    }
                    macroRunOnce = false;
                }
                tele.addData("Ended already: ", armMotor.isBusy());
                if(Math.abs(armMotor.getCurrentPosition() - armMotor.getTargetPosition()) < 30) {
                    controlMode = ControlMode.MANUAL_CONTROL;
                    macroRunOnce = true;
                }
                tele.update();
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

    private void goToPosition(int targetPosition , double power){
        armMotor.setTargetPosition(targetPosition);
        setRunToPosition();
        armMotor.setPower(power);
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

}
