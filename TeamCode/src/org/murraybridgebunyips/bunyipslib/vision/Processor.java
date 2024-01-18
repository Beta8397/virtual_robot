package org.murraybridgebunyips.bunyipslib.vision;

import org.firstinspires.ftc.vision.VisionProcessor;
import org.murraybridgebunyips.bunyipslib.vision.data.VisionData;
import org.opencv.core.Core;
import org.opencv.core.Mat;

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
     * Whether the camera stream should be processed with a vertical and horizontal flip
     */
    private boolean isFlipped;

    /**
     * Vision Processor Wrapper
     * Parameterized type T must be a subclass extension of VisionData and getName must return a non-null value
     * Remove all parameters from the constructor and replace with:
     * Super-call: {@code super([yourVisionDataClass].class)}
     *
     * @param type      the type of vision data to be processed
     *                  (must be a subclass extension of VisionData)
     * @param isFlipped whether the camera stream should be processed with a vertical and horizontal flip
     *                  (this can be changed later with .setFlipped() if needed)
     * @noinspection ConstructorNotProtectedInAbstractClass
     */
    // Public constructor as IntelliJ will auto generate a protected constructor, and it needs
    // to be public in order to be instantiated by the Vision system
    public Processor(Class<T> type, boolean isFlipped) {
        if (type == VisionData.class || !VisionData.class.isAssignableFrom(type)) {
            throw new IllegalArgumentException("Processor: T must extend VisionData");
        }
        if (getName() == null) {
            throw new IllegalArgumentException("Processor: Processor name cannot be null");
        }
        this.isFlipped = isFlipped;
    }

    @SuppressWarnings("ConstructorNotProtectedInAbstractClass")
    public Processor(Class<T> type) {
        this(type, false);
    }

    public boolean isFlipped() {
        return isFlipped;
    }

    public void setFlipped(boolean flipped) {
        isFlipped = flipped;
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
    public abstract void update();

    /**
     * Called by the vision system to process a frame
     *
     * @param frame            the frame to process
     * @param captureTimeNanos the time the frame was captured
     * @return the processed frame
     */
    public abstract Object onProcessFrame(Mat frame, long captureTimeNanos);

    @Override
    public final Object processFrame(Mat frame, long captureTimeNanos) {
        if (isFlipped)
            Core.flip(frame, frame, -1);
        return onProcessFrame(frame, captureTimeNanos);
    }
}
