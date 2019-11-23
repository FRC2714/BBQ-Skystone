package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.localization.Kinematics;
import org.firstinspires.ftc.teamcode.localization.MecanumKinematics;
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

    private DcMotorEx frontLeft, frontRight, backLeft, backRight;
    private List<ExpansionHubMotor> motors;

    private BNO055IMU imu;

    // PIDF Controllers
    private PIDFController translationalController;
    private PIDFController lateralController;
    private PIDFController headingController;

    // targets
    private Pose2d targetVelocity = new Pose2d(0.0,0.0,0.0);

    // current values
    private State currentState = State.TELEOP;
    private Pose2d currentVelocity  = new Pose2d();
    private double currentRawHeading = 0.0;
    private List<Double> lastWheelPositions;

    // constants
    private static final double TELEOP_MAX_V = 45;
    private static final double TELEOP_MAX_HEADING_V = 2;
    enum State {
        TELEOP,
        OPEN_LOOP,
        CLOSED_LOOP
    }

    private Telemetry t;

    public Drivetrain(HardwareMap map, Telemetry t) {
        this.t = t;
        this.hwMap = map;
        this.expansionHub = hwMap.get(ExpansionHubEx.class, "Expansion Hub 2");

        frontLeft = (DcMotorEx) hwMap.dcMotor.get("left_front");
        frontRight = (DcMotorEx) hwMap.dcMotor.get("right_front");
        backLeft = (DcMotorEx) hwMap.dcMotor.get("left_back");
        backRight = (DcMotorEx) hwMap.dcMotor.get("right_back");


        imu = hwMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters p = new BNO055IMU.Parameters();
        p.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(p);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        lastWheelPositions = new ArrayList<>();
    }

    @Override
    public void update() {
        bulkData = expansionHub.getBulkInputData();

        currentRawHeading = getRawHeading();
        t.addData("heading: ", Math.toDegrees(currentRawHeading));
        t.addData("normHeading: ", Math.toDegrees(norm(currentRawHeading)));
        switch(currentState) {
            case TELEOP:
                setRobotKinematics(targetVelocity, new Pose2d()); // leaving acceleration empty
                break;
        }

        t.addData("Target Velocity x: ", targetVelocity.x);
        t.addData("Target velo y: ", targetVelocity.y);
        t.addData("target velo z: ", targetVelocity.heading);
    }

    public void setTargetVelocity(Pose2d targetVelocity) {
        this.targetVelocity = targetVelocity;
    }

    private void setRobotKinematics(Pose2d velocity, Pose2d acceleration) {
        List<Double> velocities = MecanumKinematics.robotToWheelVelocites(velocity,13);
        // TODO: add accelerations and feed-forwards
        t.addData("v1: ", velocities.get(0));
        t.addData("v2: ", velocities.get(1));
        setMotorPowers(velocities.get(0), velocities.get(1),velocities.get(2), velocities.get(3));
    }

    private void setMotorPowers(double fl, double bl, double br, double fr) {
        frontLeft.setPower(fl);
        backLeft.setPower(bl);
        backRight.setPower(br);
        frontRight.setPower(fr);
    }

    public void setControllerVelocity(Pose2d target) {
        double v = target.vec().norm();
        v = Range.clip(v, -1,1) * TELEOP_MAX_V;
        t.addData("targetX: ",target.x);
        t.addData("targetY: ", target.y);
        double theta = Math.atan2(target.x, target.y);
        t.addData("theta: ", theta);
        double omega = target.heading * TELEOP_MAX_HEADING_V;
        setTargetVelocity(new Pose2d(v * Math.cos(theta), v*Math.sin(theta), omega));
    }


    // getters

    private double getRawHeading() {
        // TODO: add imu read
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;
    }

    // utilities

    // norm angle in radians
    private static double TAU = Math.PI * 2;
    private double norm(double angle) {
        double newAngle = angle % TAU;

        newAngle = (newAngle + TAU) % TAU;

        if (newAngle > Math.PI)
            newAngle -= TAU;

        return newAngle;
    }
}
