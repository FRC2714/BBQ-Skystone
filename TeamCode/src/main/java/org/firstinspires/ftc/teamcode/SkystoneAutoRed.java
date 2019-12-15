package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.utils.AutonomousRunner;
import org.firstinspires.ftc.teamcode.utils.JoystickTransform;
import org.firstinspires.ftc.teamcode.vision.VisionPipeline;
import org.firstinspires.ftc.teamcode.vision.VisionUtils;
import org.opencv.android.Utils;
import org.opencv.core.CvType;
import org.opencv.core.Mat;

import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

@Autonomous(name="SkystoneAutoRed")
public class SkystoneAutoRed extends LinearOpMode {
    Robot robot;
    JoystickTransform transform;
    AutonomousRunner scheduler;

    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = false  ;

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY =
            "AT8pGt3/////AAABmQ9LKBWthkikgQSErtn4C1GN+U/k35mErGuydnhrXtBLs2+wEnRYzMx2qJC0Q+4bHLUaWRZ18gRQcTZOoaKDYfG7yIcNfsexI4G5IdAgwfAZnSbrWco7IW2mdaHZrQ5mw/u0mh1RHbcPdK3JAheEknMP53n73JNNBFbEcB+IN2qPSI4AUrWqK3TuAl7XCnEBQrHKB7kU62rXWs+4r4/RcNB0g/yMZ3S5Yv7vfHYGMEA3/Wj+4PC/6v/pO9StgMjxKaVZMjTYiHvUN6yi6CgVfQlKlmkEMU0IR60PcUgA9hKq9CPXVNPN1tXCTGFGdd+WbhFEGdkbZxY3scU85G4kDQy2oNbFRaClpdHYINBOV1U1";

    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    private static final float mmPerInch        = 25.4f;
    private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor

    // Constant for Stone Target
    private static final float stoneZ = 2.00f * mmPerInch;

    // Constants for the center support targets
    private static final float bridgeZ = 6.42f * mmPerInch;
    private static final float bridgeY = 23 * mmPerInch;
    private static final float bridgeX = 5.18f * mmPerInch;
    private static final float bridgeRotY = 59;                                 // Units are degrees
    private static final float bridgeRotZ = 180;

    // Constants for perimeter targets
    private static final float halfField = 72 * mmPerInch;
    private static final float quadField  = 36 * mmPerInch;

    // Class Members
    private OpenGLMatrix lastLocation = null;
    private VuforiaLocalizer vuforia = null;

    /**
     * This is the webcam we are to use. As with other hardware devices such as motors and
     * servos, this device is identified using the robot configuration tool in the FTC application.
     */
    WebcamName webcamName = null;

    private boolean targetVisible = false;
    private float phoneXRotate    = 0;
    private float phoneYRotate    = 0;
    private float phoneZRotate    = 0;

    VisionPipeline p;

    private boolean shouldWrite = false;

    FtcDashboard dashboard;

    int pos = 1;

    @Override
    public void runOpMode() {
        robot = new Robot(this);
        robot.init();

        transform = new JoystickTransform();
        scheduler = new AutonomousRunner(robot);
        p = new VisionPipeline();

        dashboard = FtcDashboard.getInstance();

        vuforia = VisionUtils.getVuforiaLocalizer(hardwareMap);

        FtcDashboard dashboard = FtcDashboard.getInstance();

        Mat frame;

        while (!opModeIsActive() && !isStopRequested()) {
            VuforiaLocalizer.CloseableFrame vuFrame = null;
            if (!vuforia.getFrameQueue().isEmpty()) {
                try {
                    vuFrame = vuforia.getFrameQueue().take();
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                }

                if (vuFrame == null) continue;

                for (int i = 0; i < vuFrame.getNumImages(); i++) {
                    Image img = vuFrame.getImage(i);
                    if (img.getFormat() == PIXEL_FORMAT.RGB565) {
                        Bitmap bm = Bitmap.createBitmap(img.getWidth(), img.getHeight(), Bitmap.Config.RGB_565);
                        bm.copyPixelsFromBuffer(img.getPixels());
                        Mat mat = VisionUtils.bitmapToMat(bm, CvType.CV_8UC3);
                        Mat ret = p.processFrame(mat);
                        telemetry.addData("SKYSTONE POS: ", p.getVumarkLeftBoundary());
                        Bitmap displayBitmap = Bitmap.createBitmap(ret.width(), ret.height(), Bitmap.Config.RGB_565);
                        Utils.matToBitmap(ret, displayBitmap);
                        dashboard.sendImage(displayBitmap);
                        int vu = p.getVumarkLeftBoundary();
                        if (vu < 250) pos = 0;
                        else if (vu > 250 && vu < 620) pos = 1;
                        else pos = 2;
                    }
                }
            }
            dashboard.sendTelemetryPacket(new TelemetryPacket());
            telemetry.addData("gfsdafdsy: ",p.getVumarkLeftBoundary());
            telemetry.update();
        }

        if (opModeIsActive() && !isStopRequested()) {
            /**robot.run();
             Pose2d v = transform.transform(new Pose2d(-gamepad1.left_stick_x, -gamepad1.left_stick_y, -gamepad1.right_stick_x));
             robot.drivetrain.setControllerVelocity(v);
             telemetry.addData("x: ", -gamepad1.left_stick_y);
             telemetry.addData("y: ", -gamepad1.left_stick_x);
             telemetry.addData("heading: ", -gamepad1.right_stick_x);
             */
            scheduler.setRedBlockPath(0);
            telemetry.update();
        }

        Intake.intake = null;
        Arm.arm = null;
    }
}
