package org.murraybridgebunyips.bunyipslib.tests;

import static org.murraybridgebunyips.bunyipslib.personalitycore.CompanionCubeColours.BLUE_ELEMENT_B;
import static org.murraybridgebunyips.bunyipslib.personalitycore.CompanionCubeColours.BLUE_ELEMENT_G;
import static org.murraybridgebunyips.bunyipslib.personalitycore.CompanionCubeColours.BLUE_ELEMENT_R;
import static org.murraybridgebunyips.bunyipslib.personalitycore.CompanionCubeColours.RED_ELEMENT_B;
import static org.murraybridgebunyips.bunyipslib.personalitycore.CompanionCubeColours.RED_ELEMENT_G;
import static org.murraybridgebunyips.bunyipslib.personalitycore.CompanionCubeColours.RED_ELEMENT_R;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.murraybridgebunyips.bunyipslib.BunyipsOpMode;
import org.murraybridgebunyips.bunyipslib.Dbg;
import org.murraybridgebunyips.bunyipslib.UserSelection;
import org.murraybridgebunyips.bunyipslib.Vision;
import org.murraybridgebunyips.bunyipslib.cameras.C920;
import org.murraybridgebunyips.bunyipslib.vision.AprilTag;
import org.murraybridgebunyips.bunyipslib.vision.FtcDashboardBitmap;
import org.murraybridgebunyips.bunyipslib.vision.Processor;
import org.murraybridgebunyips.bunyipslib.vision.TFOD;
import org.murraybridgebunyips.bunyipslib.vision.TeamProp;

import java.util.ArrayList;

import kotlin.Unit;

/**
 * Test Vision processor detections and data throughput
 * Compatible with all robots with a hardware device webcam "webcam"
 *
 * @author Lucas Bubner, 2023
 */
@TeleOp(name = "Vision Test")
@Disabled
public class VisionTest extends BunyipsOpMode {
    private Vision vision;
    private Telemetry.Item i;
    private final UserSelection<Procs> proc = new UserSelection<>(this, this::callback, Procs.values());

    @SuppressWarnings("rawtypes")
    private Unit callback(Procs selection) {
        if (selection == null) {
            return Unit.INSTANCE;
        }
        ArrayList<Processor> processors = new ArrayList<>();
        switch (selection) {
            case TFOD:
                TFOD tf = new TFOD();
                processors.add(tf);
                break;
            case APRILTAG:
                AprilTag at = new AprilTag(new C920());
                processors.add(at);
                break;
            case TEAMPROP_RED:
                TeamProp tpr = new TeamProp(RED_ELEMENT_R, RED_ELEMENT_G, RED_ELEMENT_B);
                processors.add(tpr);
                break;
            case TEAMPROP_BLUE:
                TeamProp tpb = new TeamProp(BLUE_ELEMENT_R, BLUE_ELEMENT_G, BLUE_ELEMENT_B);
                processors.add(tpb);
                break;
        }
        // Always add the FtcDashboardBitmap processor
        FtcDashboardBitmap fdb = new FtcDashboardBitmap();
        processors.add(fdb);

        vision.init(processors.toArray(new Processor[0]));
        vision.start(processors.toArray(new Processor[0]));

        FtcDashboard.getInstance().startCameraStream(fdb, 0);
        i = addRetainedTelemetry("Camera Stream available.");
        return Unit.INSTANCE;
    }

    @Override
    protected boolean onInitLoop() {
        return !proc.isAlive();
    }

    @Override
    protected void onInit() {
        try {
            WebcamName webcam = (WebcamName) hardwareMap.get("webcam");
            vision = new Vision(this, webcam);
        } catch (IllegalArgumentException e) {
            Dbg.error("VisionTest is missing a webcam called 'webcam'!");
            exit();
        }
        proc.start();
    }

    @Override
    protected void onStart() {
        if (vision == null) {
            exit();
        }
        removeTelemetryItems(i);
    }

    @Override
    protected void activeLoop() {
        vision.tickAll();
        addTelemetry(String.valueOf(vision.getAllData()));
    }

    @Override
    protected void onStop() {
        FtcDashboard.getInstance().stopCameraStream();
        if (vision != null)
            vision.terminate();
    }

    private enum Procs {
        TFOD,
        APRILTAG,
        TEAMPROP_RED,
        TEAMPROP_BLUE
    }
}
