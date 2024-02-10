package org.murraybridgebunyips.bunyipslib.vision;

import android.graphics.Bitmap;
import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.murraybridgebunyips.bunyipslib.vision.data.VisionData;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.Mat;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

/**
 * Base class for all vision processors using the Vision system.
 * <p>
 * A processor will be attached to a Vision instance and will be called to process frames,
 * allowing you to access your data here using the .getData() method. This makes it useful
 * for tasks to access the latest data from the vision system, without needing to directly
 * interface with the Vision instance.
 *
 * @author Lucas Bubner, 2023
 */
public abstract class Processor<T extends VisionData> implements VisionProcessor, CameraStreamSource {

    /**
     * List of all vision data detected since the last stateful update
     */
    protected final List<T> data = Collections.synchronizedList(new ArrayList<>());

    /**
     * Bitmap for use with FtcDashboard and Bitmap processing
     */
    private final AtomicReference<Bitmap> lastFrame =
            new AtomicReference<>(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565));

    private Mat currentFrame;

    /**
     * Whether the camera stream should be processed with a vertical and horizontal flip
     */
    private boolean isFlipped;

    private boolean isAttached;

    /**
     * Determine whether the processor is attached to a Vision instance.
     * Checking this is useful for processors that have been passed into tasks but cannot
     * be checked by looking directly at the vision system.
     */
    public boolean isAttached() {
        return isAttached;
    }

    // Package-private, set internally by Processor
    void setAttached(boolean attached) {
        isAttached = attached;
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
        synchronized (data) {
            return data;
        }
    }

    /**
     * Called to update new data from the vision system, which involves interpreting,
     * collecting, or otherwise processing new vision data per frame. This method should
     * refresh `this.data` with the latest information from the vision system to be accessed
     * with your methods on .getData().T (your VisionData class). `this.data` is automatically
     * cleared upon each iteration, so opt to using realtime data in this method.
     * This method will be called automatically once attached to a Vision instance.
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
    public final Object processFrame(Mat f, long captureTimeNanos) {
        Mat frame = f.clone();
        if (isFlipped)
            Core.flip(frame, frame, -1);
        Object procFrame = onProcessFrame(frame, captureTimeNanos);
        Bitmap b = Bitmap.createBitmap(frame.width(), frame.height(), Bitmap.Config.RGB_565);
        Utils.matToBitmap(frame, b);
        lastFrame.set(b);
        synchronized (data) {
            data.clear();
            update();
        }
        return procFrame;
    }

    @Override
    public final void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        Bitmap f = lastFrame.get();
        // Link bitmap to canvas
        Canvas linkedCanvas = new Canvas(f);
        onFrameDraw(linkedCanvas, onscreenWidth, onscreenHeight, scaleBmpPxToCanvasPx, scaleCanvasDensity, userContext);
        // Copy back into the canvas for rendering
        // On FtcDashboard this causes any drawFrames to flicker, but this is not because it is losing tracking
        // TODO: test if the Control Hub will still render drawFrames even if the RC screen technically does not exist
        canvas.drawBitmap(f, 0, 0, null);
    }

    public abstract void onFrameDraw(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext);

    @Override
    public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {
        continuation.dispatch(bitmapConsumer -> bitmapConsumer.accept(lastFrame.get()));
    }
}
