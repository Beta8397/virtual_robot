package Autonomous;

import Misc.Log;
import virtual_robot.config.Config;
import virtual_robot.config.UltimateGoal;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

/**
 * Created by robotics on 12/18/18.
 */

public class VisionHelperUltimateGoal extends Thread {
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_QUAD_STACK = "Quad";
    private static final String LABEL_SINGLE_STACK = "Single";
    public final static int SLEEP_TIME_MILLIS = 200;
    public final static int PHONE_CAMERA = 0;
    public final static int WEBCAM = 1;
    public final static int LOCATION = 0, RING_DETECTION = 1, BOTH = 2;
    VuforiaLocalizer vuforia;
    VuforiaTrackables targetsUltimateGoal;
    VuforiaTrackable blueTowerGoalTarget;
    VuforiaTrackable redTowerGoalTarget;
    VuforiaTrackable redAllianceTarget;
    VuforiaTrackable blueAllianceTarget;
    VuforiaTrackable frontWallTarget;
    private volatile TFObjectDetector tfod;
    private volatile boolean running = true, trackingLocation = false;
    private volatile boolean findingRings = false;
    private volatile boolean targetVisible = false;
    private volatile OpenGLMatrix lastLocation = null;
    private volatile OpenGLMatrix lastRingLocation = null;
    private volatile Location robotLocation = new Location(0, 0);
    private volatile Location ringLocation = new Location(0, 0);
    private volatile Orientation ringOrientation;
    List<VuforiaTrackable> allTrackables;
    Orientation robotOrientation;
    VectorF translation;
    VectorF ringTranslation;
    private int mode = LOCATION;
    private int numOfRings = 0;
    private RevBlinkinLedDriver LEDStripController;

    private static final float mmPerInch        = 25.4f;
    private static final float mmFTCFieldWidth  = (12*6) * mmPerInch;  // the width of the FTC field (from the center point to the outer panels)
    private static final float mmTargetHeight   = (5.75f) * mmPerInch;    // the height of the center of the target image above the floor

    // Constants for perimeter targets
    private static final float halfField = 72 * mmPerInch;
    private static final float quadField  = 36 * mmPerInch;

    final int CAMERA_FORWARD_DISPLACEMENT_FROM_CENTER = (int)(1.5*mmPerInch);
    final int CAMERA_VERTICAL_DISPLACEMENT_FROM_CENTER = (int)(14*mmPerInch);
    final int CAMERA_LEFT_DISPLACEMENT_FROM_CENTER = (int)(-mmPerInch);

    public VisionHelperUltimateGoal(int camera, HardwareMap hardwareMap) { this(camera, BOTH, hardwareMap); }

    public VisionHelperUltimateGoal(int camera, int mode, HardwareMap hardwareMap) {
        this.mode = mode;
        switch (mode) {
            case LOCATION:
//                vuforia = VuforiaHelper.initVuforia(camera, hardwareMap);
                break;
            case RING_DETECTION:
            case BOTH:
                initBoth(camera, hardwareMap);
                break;
            default:
                break;
        }

        LEDStripController = hardwareMap.get(RevBlinkinLedDriver.class, "LEDStripController");
        LEDStripController.resetDeviceConfigurationForOpMode();
        LEDStripController.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
    }

    private void initBoth(int camera, HardwareMap hardwareMap) {
//        try {
//            vuforia = VuforiaHelper.initVuforia(camera, hardwareMap);
//            int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
//                    "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//            TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
//            tfodParameters.minResultConfidence = 0.8f;
//            tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
//            tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_QUAD_STACK, LABEL_SINGLE_STACK);
//        } catch (Exception e) {
//            Log.e("VisionHelper Error", e.toString());
//            throw new RuntimeException(e);
//        }
    }

    public void setLEDMode(RevBlinkinLedDriver.BlinkinPattern pattern) {
        LEDStripController.setPattern(pattern);
    }

    @Override
    public void run() {
//        if(findingRings) cycleRingDetection();
//        while (running) {
//            if (trackingLocation) updateRobotLocation();
//                if (findingSkyStone) getStonesInView();
//            if (findingRings) countRings();
//            try {
//                sleep(SLEEP_TIME_MILLIS);
//            } catch (InterruptedException e) {
//                e.printStackTrace();
//            }
//        }
    }

    public void startDetection() {
        loadNavigationAssets();
        robotOrientation = new Orientation(EXTRINSIC, XYZ, DEGREES, 0, 0, 0, 0);
        running = true;
        findingRings = true;
        this.start();
    }

    public void stopDetection() {
        findingRings = false;
        running = false;
    }

    public void startFindingRings() { findingRings = true; }

    public void startTrackingLocation() {
        trackingLocation = true;
    }

    public void stopTrackingLocation() { trackingLocation = false; }

//    public Recognition[] getRingsInView() {
//        return tfod.getRecognitions().toArray(new Recognition[0]);
//    }

    public void countRings() {
        numOfRings = ((UltimateGoal)Config.GAME).getStarterStackSize();
    }

//    public void cycleRingDetection() {
//        for(int i = 0; i < 25; i++) {
//            getRingsInView();
//        }
//    }

    public RingCount getRingCount() {
        switch (numOfRings) {
            case 1:
                return RingCount.SINGLE_STACK;
            case 4:
                return RingCount.QUAD_STACK;
        }
        return RingCount.NO_RINGS;
    }

    public int numOfSeenRings() { return numOfRings; }

    public Orientation getRobotOrientation() {
        return robotOrientation;
    }

    public double getRobotHeading() {
//        double heading = -robotOrientation.thirdAngle;
//        return heading;
        return -robotOrientation.thirdAngle;
    }

    public Location getRobotLocation() {
        if(trackingLocation) return robotLocation;
        else return null;
    }

    private void updateRobotLocation() {
//        targetVisible = false;
//        for (VuforiaTrackable trackable : allTrackables) {
//            if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
//                targetVisible = true;
//
//                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
//                if (robotLocationTransform != null) {
//                    lastLocation = robotLocationTransform;
//                }
//                break;
//            }
//        }
//
//        if (targetVisible) {
//            translation = lastLocation.getTranslation();
//            robotLocation = new Location(0, 0);
//            robotLocation.update(translation.get(0) / mmPerInch, translation.get(1) / mmPerInch);
//            robotOrientation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
//        } else {
//            robotLocation = null;
//            robotOrientation = null;
//        }
    }

    public void loadNavigationAssets() {
//        targetsUltimateGoal = vuforia.loadTrackablesFromAsset("UltimateGoal"); // NOTE: the asset is titled Skystone not SkyStone... this is why I told you to copy and paste...
//        blueTowerGoalTarget = targetsUltimateGoal.get(0);
//        blueTowerGoalTarget.setName("Blue Tower Goal Target");
//        redTowerGoalTarget = targetsUltimateGoal.get(1);
//        redTowerGoalTarget.setName("Red Tower Goal Target");
//        redAllianceTarget = targetsUltimateGoal.get(2);
//        redAllianceTarget.setName("Red Alliance Target");
//        blueAllianceTarget = targetsUltimateGoal.get(3);
//        blueAllianceTarget.setName("Blue Alliance Target");
//        frontWallTarget = targetsUltimateGoal.get(4);
//        frontWallTarget.setName("Front Wall Target");
//
//        allTrackables = new ArrayList<>();
//        allTrackables.addAll(targetsUltimateGoal);
//
//        redAllianceTarget.setLocation(OpenGLMatrix
//                .translation(0, -halfField, mmTargetHeight)
//                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));
//
//        blueAllianceTarget.setLocation(OpenGLMatrix
//                .translation(0, halfField, mmTargetHeight)
//                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));
//        frontWallTarget.setLocation(OpenGLMatrix
//                .translation(-halfField, 0, mmTargetHeight)
//                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 90)));
//
//        // The tower goal targets are located a quarter field length from the ends of the back perimeter wall.
//        blueTowerGoalTarget.setLocation(OpenGLMatrix
//                .translation(halfField, quadField, mmTargetHeight)
//                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , -90)));
//        redTowerGoalTarget.setLocation(OpenGLMatrix
//                .translation(halfField, -quadField, mmTargetHeight)
//                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));
//
//        OpenGLMatrix cameraLocationOnRobot = OpenGLMatrix
//                .translation(CAMERA_FORWARD_DISPLACEMENT_FROM_CENTER, CAMERA_LEFT_DISPLACEMENT_FROM_CENTER, CAMERA_VERTICAL_DISPLACEMENT_FROM_CENTER)
//                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES,
//                        90, 0, 90));
//
//        for (VuforiaTrackable trackable : allTrackables)
//            ((VuforiaTrackableDefaultListener)trackable.getListener()).setCameraLocationOnRobot(vuforia.getCameraName(), cameraLocationOnRobot);
//
//        targetsUltimateGoal.activate();
//        if (tfod != null) {
//            tfod.activate();
//        }
    }

    public void kill() {
//        stopDetection();
//        targetsUltimateGoal.deactivate();
//        if(mode == BOTH || mode == RING_DETECTION) tfod.shutdown();
//        VuforiaHelper.kill();
    }
}
