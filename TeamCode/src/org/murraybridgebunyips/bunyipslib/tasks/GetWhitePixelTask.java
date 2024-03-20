package org.murraybridgebunyips.bunyipslib.tasks;

import org.murraybridgebunyips.bunyipslib.tasks.bases.NoTimeoutTask;
import org.murraybridgebunyips.bunyipslib.vision.Vision;
import org.murraybridgebunyips.bunyipslib.vision.data.TfodData;
import org.murraybridgebunyips.bunyipslib.vision.processors.TFOD;

import java.util.List;

/**
 * Task for obtaining the White Pixel randomisation mark for the autonomous period.
 * This task is to be run when the camera is facing the randomisation mark, and can be used
 * to determine if there is a Pixel in front of the camera or not. Paired with the result of the
 * task, the OpMode can map where the Pixel will be.
 * FTC 2023-2024 CENTERSTAGE
 *
 * @author Lucas Bubner, 2023
 */
public class GetWhitePixelTask extends NoTimeoutTask {
    /**
     * For use in CAPTURE mode, lock in the spike detection if it is detected for this many frames
     */
    public static final int SPIKE_FRAME_THRESHOLD = 5;
    /**
     * For use in CAPTURE mode, lock in the spike detection if it is detected with this confidence
     */
    public static final double SPIKE_CONFIDENCE_THRESHOLD = 0.95;
    private final Vision vision;
    private final TFOD tfod;
    private final Aggression aggression;
    private int spikeFrames;
    private double confidence;
    /**
     * Updated by the task to indicate if a spike has been found
     */
    private volatile boolean foundSpike;

    /**
     * Assumes tfod has already been initialised with vision.init(tfod) in the OpMode
     *
     * @param vision     Vision management class instance
     * @param tfod       VisionProcessor instance for TFOD
     * @param aggression How decisive the task should be in determining if a spike has been found
     */
    public GetWhitePixelTask(Vision vision, TFOD tfod, Aggression aggression) {
        this.vision = vision;
        this.tfod = tfod;
        this.aggression = aggression;
    }

    /**
     * @return Whether a spike mark (white pixel) has been found.
     */
    public boolean hasFoundSpike() {
        return foundSpike;
    }

    @Override
    protected void init() {
        // We are using the default settings of TFOD for white spike detection
        // We assume that the OpMode has opened a new VisionPortal for us with .init()
        try {
            vision.start(tfod);
        } catch (IllegalStateException e) {
            // OpMode did not start the VisionPortal for us, we better do it ourselves
            opMode.log("WARNING: TFOD processor not initialised by Vision.init()! Initialising now...");
            vision.init(tfod);
            vision.start(tfod);
        }
    }

    @Override
    protected boolean isTaskFinished() {
        switch (aggression) {
            case INSTANT:
                return foundSpike;
            case CAPTURE:
                if (foundSpike) {
                    spikeFrames++;
                } else {
                    spikeFrames = 0;
                }
                return (spikeFrames >= SPIKE_FRAME_THRESHOLD && confidence >= SPIKE_CONFIDENCE_THRESHOLD);
            case TIMEOUT:
            default:
                return false;
        }
    }

    @Override
    protected void periodic() {
        tfod.update();
        List<TfodData> tfodData = tfod.getData();
        if (tfodData.isEmpty()) {
            foundSpike = false;
            return;
        }
        for (TfodData data : tfodData) {
            if (!data.getLabel().equals("Spike")) {
                return;
            }
            foundSpike = true;
            confidence = data.getConfidence();
        }
    }

    @Override
    protected void onFinish() {
        // We will not need TFOD anymore
        vision.stop(tfod);
    }

    enum Aggression {
        // Immediately report as finished if a spike is detected for one frame
        INSTANT,
        // Report as finished if a spike is detected under a set of criteria (confidence, frames)
        CAPTURE,
        // Never report as finished until the timeout, result will be determined at the end of the timeout
        TIMEOUT
    }
}
