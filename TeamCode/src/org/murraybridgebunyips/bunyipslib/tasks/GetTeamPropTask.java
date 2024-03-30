package org.murraybridgebunyips.bunyipslib.tasks;

import org.murraybridgebunyips.bunyipslib.Dbg;
import org.murraybridgebunyips.bunyipslib.Direction;
import org.murraybridgebunyips.bunyipslib.tasks.bases.ForeverTask;
import org.murraybridgebunyips.bunyipslib.vision.data.ContourData;
import org.murraybridgebunyips.bunyipslib.vision.processors.ColourThreshold;

/**
 * Task to get the position of the team prop.
 *
 * @author Lucas Bubner, 2024
 */
public class GetTeamPropTask extends ForeverTask {
    private final ColourThreshold colourThreshold;
    private volatile Direction position = Direction.LEFT;

    /**
     * Create a new GetTeamPropTask.
     *
     * @param colourThreshold the initialised and running colour threshold processor
     */
    public GetTeamPropTask(ColourThreshold colourThreshold) {
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
            Dbg.log(biggestContour.getYaw());
            position = biggestContour.getYaw() > 0.5 ? Direction.RIGHT : Direction.FORWARD;
        }
    }

    @Override
    protected void onFinish() {
        // no-op
    }
}
