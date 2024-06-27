package org.murraybridgebunyips.bunyipslib.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.murraybridgebunyips.bunyipslib.BunyipsOpMode;
import org.murraybridgebunyips.bunyipslib.EmergencyStop;
import org.murraybridgebunyips.bunyipslib.Threads;
import org.murraybridgebunyips.bunyipslib.UserSelection;
import org.murraybridgebunyips.bunyipslib.vision.Processor;
import org.murraybridgebunyips.bunyipslib.vision.Vision;
import org.murraybridgebunyips.bunyipslib.vision.processors.AprilTag;
import org.murraybridgebunyips.bunyipslib.vision.processors.TFOD;
import org.murraybridgebunyips.bunyipslib.vision.processors.centerstage.WhitePixel;

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
    private Telemetry.Item cameraStreamNotification;
    private final UserSelection<Procs> procChooser = new UserSelection<>(this, this::callback, Procs.values());

    @SuppressWarnings("rawtypes")
    private void callback(Procs selection) {
        if (selection == null) {
            vision.init(vision.raw);
            vision.start(vision.raw);
            vision.startPreview();
            return;
        }
        Processor chosenProcessor = null;
        switch (selection) {
            case TFOD:
                chosenProcessor = new TFOD();
                break;
            case APRILTAG:
                chosenProcessor = new AprilTag();
                break;
            case WHITE_PIXEL:
                chosenProcessor = new WhitePixel();
                break;
        }

        vision.init(chosenProcessor, vision.raw);
        vision.start(chosenProcessor, vision.raw);
        vision.startPreview();

        cameraStreamNotification = telemetry.addRetained("Camera Stream available.").getItem();
    }

    @Override
    protected boolean onInitLoop() {
        return !Threads.isRunning(procChooser);
    }

    @Override
    protected void onInit() {
        try {
            WebcamName webcam = (WebcamName) hardwareMap.get("webcam");
            vision = new Vision(webcam);
        } catch (IllegalArgumentException e) {
            throw new EmergencyStop("VisionTest is missing a webcam called 'webcam'!");
        }
        Threads.start(procChooser);
    }

    @Override
    protected void onStart() {
        if (vision == null) {
            exit();
        }
        telemetry.remove(cameraStreamNotification);
    }

    @Override
    protected void activeLoop() {
        telemetry.add(String.valueOf(vision.getAllData()));
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
        WHITE_PIXEL
    }
}
