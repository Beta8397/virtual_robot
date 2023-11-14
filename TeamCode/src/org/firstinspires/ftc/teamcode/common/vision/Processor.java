package org.firstinspires.ftc.teamcode.common.vision;

import org.firstinspires.ftc.teamcode.common.vision.data.VisionData;
import org.firstinspires.ftc.vision.VisionProcessor;

import java.util.ArrayList;
import java.util.List;

/**
 * Base class for all vision processors using the Vision system
 *
 * @author Lucas Bubner, 2023
 */
public abstract class Processor<T extends VisionData> implements VisionProcessor {

    /**
     * List of all vision data detected since the last stateful update
     */
    protected final List<T> data = new ArrayList<>();

    /**
     * Vision Processor Wrapper
     * Parameterized type T must be a subclass extension of VisionData and getName must return a non-null value
     * Super-call: {@code super([yourVisionDataClass].class)}
     */
    protected Processor(Class<T> type) {
        if (type == VisionData.class || !VisionData.class.isAssignableFrom(type)) {
            throw new IllegalArgumentException("Processor: T must extend VisionData");
        }
        if (getName() == null) {
            throw new IllegalArgumentException("Processor: Processor name cannot be null");
        }
    }

    /**
     * Unique identifier for the processor
     */
    public abstract String getName();

    /**
     * Get the list of vision data
     *
     * @return list of all vision data detected since the last stateful update
     */
    public List<T> getData() {
        return data;
    }

    /**
     * Called to update new data from the vision system, which involves interpreting,
     * collecting, or otherwise processing new vision data per frame. This method should
     * refresh `this.data` with the latest information from the vision system to be accessed
     * with your methods on .getData().T (your VisionData class).
     */
    public abstract void tick();
}
