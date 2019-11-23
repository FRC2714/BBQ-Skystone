package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
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
import java.util.Arrays;
import java.util.List;

public class Drivetrain implements Subsystem {

    private static final int TICKS_PER_INCH = 30;
    private ExpansionHubEx expansionHub;
    private RevBulkData bulkData;
    private HardwareMap hwMap;

    private ExpansionHubMotor frontLeft, frontRight, backLeft, backRight;
    private List<ExpansionHubMotor> motors;

    private BNO055IMU imu;

    // Coeffecients
    PIDCoefficients translationalCoeffecients = new PIDCoefficients(0.8,0,0);
    PIDCoefficients lateralCoeffecients = new PIDCoefficients(0.5,0,0);
    PIDCoefficients headingCoeffecients = new PIDCoefficients(0.9,0,0);

    // PIDF Controllers
    private PIDFController translationalController = new PIDFController(translationalCoeffecients, 0.0, 0,0);
    private PIDFController lateralController = new PIDFController(lateralCoeffecients, 0.0,0.0,0.0);
    private PIDFController headingController = new PIDFController(headingCoeffecients, 0.225, 0.0,0.0);

    // targets
    private Pose2d targetVelocity = new Pose2d(0.0,0.0,0.0); // MAKE SURE THIS STARTS AS 0

    // current values
    private State currentState = State.CLOSED_LOOP;
    private Pose2d currentVelocity  = new Pose2d();
    private double currentRawHeading = 0.0;
    private Pose2d currentEstimatedPose = new Pose2d();
    private List<Double> currentWheelPositions;

    // previous values (for deltas)
    private List<Double> lastWheelPositions;
    private double lastRawHeading = 0.0;
    private Pose2d targetPose = new Pose2d(100,0,0);

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

        frontLeft = (ExpansionHubMotor) hwMap.dcMotor.get("left_front");
        frontRight = (ExpansionHubMotor) hwMap.dcMotor.get("right_front");
        backLeft = (ExpansionHubMotor) hwMap.dcMotor.get("left_back");
        backRight = (ExpansionHubMotor) hwMap.dcMotor.get("right_back");

        imu = hwMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters p = new BNO055IMU.Parameters();
        p.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(p);

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

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
        currentWheelPositions = new ArrayList<>();
    }

    @Override
    public void update() {
        bulkData = expansionHub.getBulkInputData();
        currentRawHeading = getRawHeading();
        currentWheelPositions = getWheelPositions();
        currentEstimatedPose = getEstimatedPose();
        t.addData("heading: ", Math.toDegrees(currentRawHeading));
        t.addData("normHeading: ", Math.toDegrees(norm(currentRawHeading)));
        
        switch(currentState) {
            case TELEOP:
                setRobotKinematics(targetVelocity, new Pose2d()); // leaving acceleration empty
                break;
            case CLOSED_LOOP:
                translationalController.targetPosition = targetPose.x - currentEstimatedPose.x;
                lateralController.targetPosition = targetPose.y - currentEstimatedPose.y;
                headingController.targetPosition = targetPose.heading - currentEstimatedPose.heading;

                double translationalCorrection = translationalController.update(0,targetVelocity.x,0.0);
                double lateralCorrection = lateralController.update(0,targetVelocity.y, 0.0);
                double headingCorrection = headingController.update(0,targetVelocity.heading,0.0);

                Pose2d velocityCorrection = new Pose2d(translationalCorrection, lateralCorrection, headingCorrection);
                setRobotKinematics(targetVelocity.plus(velocityCorrection), new Pose2d());
        }

        t.addData("Target Velocity x: ", targetVelocity.x);
        t.addData("Target velo y: ", targetVelocity.y);
        t.addData("target velo z: ", targetVelocity.heading);

        List<Integer> wheelCounts = getWheelCounts();
        t.addData("FL Counts: ", wheelCounts.get(0));
        t.addData("BL Counts: ", wheelCounts.get(1));
        t.addData("BR Counts: ", wheelCounts.get(2));
        t.addData("FR Counts: ", wheelCounts.get(3));

        lastRawHeading = currentRawHeading;
        lastWheelPositions = currentWheelPositions;

        t.addData("X: ", currentEstimatedPose.x);
        t.addData("Y: ", currentEstimatedPose.y);
        t.addData("heading: ", currentEstimatedPose.heading);
    }

    public void setTargetVelocity(Pose2d targetVelocity) {
        this.targetVelocity = targetVelocity;
    }

    private void setRobotKinematics(Pose2d velocity, Pose2d acceleration) {
        List<Double> velocities = MecanumKinematics.robotToWheelVelocites(velocity,13);
        // TODO: add accelerations and feed-forwards
        t.addData("fl velocity: ", velocities.get(0));
        t.addData("bl velocity: ", velocities.get(1));
        t.addData("br velocity: ", velocities.get(2));
        t.addData("fr velocities: ", velocities.get(3));
        List<Double> powers = Kinematics.calculateMotorFeedforward(velocities, new ArrayList<>(), 0.0225, 0.0,0.0);
        t.addData("fl power: ", powers.get(0));
        t.addData("bl powers: ", powers.get(1));
        t.addData("br powers: ",powers.get(2));
        t.addData("fr powers: ", powers.get(3));
        setMotorPowers(powers.get(0), powers.get(1),powers.get(2), powers.get(3));
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

    private List<Integer> getWheelCounts() {
        if (bulkData == null) return Arrays.asList(0,0,0,0);
        return Arrays.asList(bulkData.getMotorCurrentPosition(frontLeft),bulkData.getMotorCurrentPosition(backLeft),
                bulkData.getMotorCurrentPosition(backRight), bulkData.getMotorCurrentPosition(frontRight));
    }

    private List<Double> getWheelPositions() {
        if (bulkData == null) return Arrays.asList(0.0,0.0,0.0,0.0);
        double flInches = bulkData.getMotorCurrentPosition(frontLeft) / TICKS_PER_INCH;
        double blInches = bulkData.getMotorCurrentPosition(backLeft) / TICKS_PER_INCH;
        double brInches = bulkData.getMotorCurrentPosition(backRight) / TICKS_PER_INCH;
        double frInches = bulkData.getMotorCurrentPosition(frontRight) / TICKS_PER_INCH;

        return Arrays.asList(flInches, blInches, brInches, frInches);
    }

    public Pose2d getEstimatedPose() {
        if (!lastWheelPositions.isEmpty()) {
            List<Double> wheelDeltas = new ArrayList<>();
            for (int i = 0; i < currentWheelPositions.size(); i++) wheelDeltas.add(currentWheelPositions.get(i) - lastWheelPositions.get(i));
            Pose2d poseDelta = MecanumKinematics.wheelToRobotVelocity(wheelDeltas, 13);
            double headingDelta = norm(currentRawHeading - lastRawHeading);
            return Kinematics.odometryUpdate(currentEstimatedPose, new Pose2d(poseDelta.vec(), headingDelta));
        }
        return currentEstimatedPose;
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
