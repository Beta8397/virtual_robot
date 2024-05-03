package org.murraybridgebunyips.bunyipslib.tasks;

import org.murraybridgebunyips.bunyipslib.Direction;
import org.murraybridgebunyips.bunyipslib.tasks.bases.ForeverTask;
import org.murraybridgebunyips.bunyipslib.vision.data.ContourData;
import org.murraybridgebunyips.bunyipslib.vision.processors.ColourThreshold;

/**
 * A generic task to get the position of a contour based on three positions, where the contour in question
 * may be viewed in the two halves of the camera frame on the FORWARD and RIGHT sides.
 * <p>
 * An example of this task is use with CENTERSTAGE's Team Prop detection, where the camera will face the Center and Right
 * Spike Marks, where if the prop is on the left of the camera, it is in the center. If no contour is detected, the
 * prop is assumed to be on the left as it cannot be seen.
 *
 * @author Lucas Bubner, 2024
 */
public class GetTriPositionContourTask extends ForeverTask {
    private final ColourThreshold colourThreshold;
    private volatile Direction position = Direction.LEFT;

    /**
     * Create a new GetTriPositionContourTask.
     *
     * @param colourThreshold the initialised and running colour threshold processor
     */
    public GetTriPositionContourTask(ColourThreshold colourThreshold) {
        this.colourThreshold = colourThreshold;
    }

    public Direction getPosition() {
        return position;
    }

    @Override
    protected void init() {
        if (!colourThreshold.isRunning()) {
            throw new IllegalStateException("Processor not attached and running on an active vision processor");
        }
    }

    @Override
    protected void periodic() {
        ContourData biggestContour = ContourData.getLargest(colourThreshold.getData());
        if (biggestContour != null) {
            position = biggestContour.getYaw() > 0.5 ? Direction.RIGHT : Direction.FORWARD;
        }
    }

    @Override
    protected void onFinish() {
        // no-op
    }
}
