package org.murraybridgebunyips.bunyipslib.tasks;

import static org.murraybridgebunyips.bunyipslib.external.units.Units.Seconds;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.murraybridgebunyips.bunyipslib.Direction;
import org.murraybridgebunyips.bunyipslib.external.units.Measure;
import org.murraybridgebunyips.bunyipslib.external.units.Time;
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
    private final ElapsedTime lockoutTimer = new ElapsedTime();
    private Measure<Time> leftSidePersistenceTime = Seconds.of(3);
    private volatile Direction position = Direction.LEFT;
    private Telemetry.Item item;

    /**
     * Create a new GetTriPositionContourTask.
     *
     * @param colourThreshold the initialised and running colour threshold processor
     */
    public GetTriPositionContourTask(ColourThreshold colourThreshold) {
        this.colourThreshold = colourThreshold;
        withName("Get Tri Position Contour");
    }

    /**
     * Set the time for the left side to be persistent before the position is set to LEFT.
     *
     * @param time the time for the left side to be persistent before the position is set back to LEFT from being RIGHT
     *             or FORWARD
     * @return this
     */
    public GetTriPositionContourTask withLeftSidePersistenceTime(Measure<Time> time) {
        leftSidePersistenceTime = time;
        return this;
    }

    @NonNull
    public Direction getPosition() {
        return position;
    }

    private String buildString() {
        return "Currently detecting position: " + position;
    }

    @Override
    protected void init() {
        if (!colourThreshold.isRunning()) {
            throw new IllegalStateException("Processor not attached and running on an active vision processor");
        }
        item = opMode.telemetry.addRetained(buildString());
    }

    @Override
    protected void periodic() {
        ContourData biggestContour = ContourData.getLargest(colourThreshold.getData());
        if (biggestContour != null) {
            position = biggestContour.getYaw() > 0.5 ? Direction.RIGHT : Direction.FORWARD;
            lockoutTimer.reset();
        } else if (lockoutTimer.seconds() >= leftSidePersistenceTime.in(Seconds)) {
            position = Direction.LEFT;
        }
        item.setValue(buildString());
    }

    @Override
    protected void onFinish() {
        opMode.telemetry.remove(item);
    }
}
