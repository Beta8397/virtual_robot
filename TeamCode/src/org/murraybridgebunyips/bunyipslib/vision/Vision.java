package org.murraybridgebunyips.bunyipslib.vision;


import android.graphics.Canvas;
import android.util.Size;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.murraybridgebunyips.bunyipslib.BunyipsComponent;
import org.murraybridgebunyips.bunyipslib.Threads;
import org.murraybridgebunyips.bunyipslib.vision.data.VisionData;
import org.opencv.core.Mat;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Objects;

/**
 * Component wrapper to support the v8.2+ SDK's included libraries for Camera operation.
 * This is an expansible system to run Processor components using the VisionPortal.
 * <p>
 * You will pass your own processors that you manage, and Vision will handle the data collection.
 * <p>
 * Vision is not a traditional subsystem, as it runs on another thread and updates are
 * managed at the discretion of the VisionPortal. Once set up, Vision will automatically
 * manage the camera stream and defined processor updates. All you will need to do is collect
 * the data from the processors and use it in your OpMode.
 *
 * @author Lucas Bubner, 2023
 */
@Config
public class Vision extends BunyipsComponent {
    /**
     * A built-in raw feed Processor that will do nothing but provide the raw camera feed.
     * Useful for debugging and testing, pass this raw field (vision.raw) to init() and start() to use it.
     */
    public Raw raw = new Raw();
    public static int CAMERA_WIDTH = 640;
    public static int CAMERA_HEIGHT = 480;
    @SuppressWarnings("rawtypes")
    private final List<Processor> processors = new ArrayList<>();
    private final CameraName camera;
    private VisionPortal visionPortal;
    private SwitchableVisionSender visionSender;

    public Vision(CameraName camera, int cameraWidth, int cameraHeight) {
        this.camera = camera;
        // Allow the user to set the camera resolution if they want
        CAMERA_WIDTH = cameraWidth;
        CAMERA_HEIGHT = cameraHeight;
    }

    public Vision(CameraName camera) {
        this.camera = camera;
    }

    /**
     * Get all VisionProcessors attached to the VisionPortal.
     */
    @SuppressWarnings("rawtypes")
    public List<Processor> getAttachedProcessors() {
        return processors;
    }

    /**
     * Initialises the Vision class with the specified processors.
     * This method should only be called once per OpMode. Additional calls will internally
     * terminate the VisionPortal and reinitialise it with the new processors (this is a highly expensive operation).
     * Processors will be STOPPED by default, you must call {@code start()} after initialising.
     *
     * @param processors Processor instances
     */
    @SuppressWarnings("rawtypes")
    public void init(Processor... processors) {
        if (visionPortal != null) {
            opMode.log("WARNING: Vision already initialised! Tearing down...");
            terminate();
        }

        if (processors.length == 0) {
            throw new IllegalArgumentException("Vision: Must initialise at least one integrated processor!");
        }

        // Hand over instance control to the VisionPortal
        this.processors.addAll(Arrays.asList(processors));

        // Initialise the VisionPortal with our newly created processors
        VisionPortal.Builder builder = new VisionPortal.Builder();
        for (Processor processor : this.processors) {
            if (processor == null) {
                throw new IllegalStateException("Vision: Processor is not instantiated!");
            }
            if (processor.getName() == null) {
                throw new IllegalStateException("Vision: Processor name cannot be null!");
            }
            for (Processor otherProcessor : this.processors) {
                if (otherProcessor != processor && otherProcessor.getName().equals(processor.getName())) {
                    throw new IllegalStateException("Vision: Processor name must be unique!");
                }
            }
            builder.addProcessor(processor);
            processor.setAttached(true);
            opMode.log("vision processor '%' initialised.", processor.getName());
        }

        visionPortal = builder
                .setCamera(camera)
                .setCameraResolution(new Size(CAMERA_WIDTH, CAMERA_HEIGHT))
                // Live view needs to be enabled to allow for drawFrame() to work for FtcDashboard
                // for integrated processors such as TFOD and AprilTag as they don't like to work
                // outside of the live view environment
                .enableLiveView(true)
                // Set any additional VisionPortal settings here
                .build();

        // Disable the vision processors by default. The OpMode must call start() to enable them.
        for (Processor processor : this.processors) {
            visionPortal.setProcessorEnabled(processor, false);
        }

        opMode.log("visionportal ready.");
    }

    /**
     * Start desired processors. This method must be called before trying to extract data from
     * the cameras, and must be already initialised with the init() method.
     *
     * @param processors Processor instances
     */
    @SuppressWarnings("rawtypes")
    public void start(Processor... processors) {
        if (visionPortal == null) {
            throw new IllegalStateException("Vision: VisionPortal is not initialised from init()!");
        }

        // Resume the stream if it was previously stopped or is not running
        if (visionPortal.getCameraState() == VisionPortal.CameraState.CAMERA_DEVICE_READY ||
                visionPortal.getCameraState() == VisionPortal.CameraState.STOPPING_STREAM) {
            // Note if the camera state is STOPPING_STREAM, it will block the thread until the
            // stream is resumed. This is a documented operation in the SDK.
            opMode.log("visionportal restarting...");
            visionPortal.resumeStreaming();
        }

        for (Processor processor : processors) {
            if (processor == null) {
                throw new IllegalStateException("Vision: Processor is not instantiated!");
            }
            if (!this.processors.contains(processor)) {
                throw new IllegalStateException("Vision: Tried to start a processor that was not initialised!");
            }
            visionPortal.setProcessorEnabled(processor, true);
            opMode.log("vision processor '%' started.", processor.getName());
        }
    }

    /**
     * Stop desired processors (Level 2).
     * <p>
     * This method should be called when hardware resources no longer
     * need to be allocated to operating the cameras, and should have the option to be re-enabled
     * with start().
     * <p>
     * Note: The VisionPortal is automatically closed at the end of the OpMode's run time, calling
     * stop() or terminate() is not required at the end of an OpMode.
     * <p>
     * Passing no arguments will pause the Camera Stream (Level 3). Pausing
     * the camera stream will automatically disable any running processors. Note this may
     * take some very small time to resume the stream if start() is called again. If you don't plan
     * on using the camera stream again, it is recommended to call terminate() instead.
     *
     * @param processors Processor instances
     */
    @SuppressWarnings("rawtypes")
    public void stop(Processor... processors) {
        if (visionPortal == null) {
            throw new IllegalStateException("Vision: VisionPortal is not initialised from init()!");
        }

        // Disable processors without pausing the stream
        for (Processor processor : processors) {
            if (processor == null) {
                throw new IllegalStateException("Vision: Processor is not instantiated!");
            }
            if (!this.processors.contains(processor)) {
                throw new IllegalStateException("Vision: Tried to stop a processor that was not initialised!");
            }
            visionPortal.setProcessorEnabled(processor, false);
            opMode.log("vision processor '%' paused.", processor.getName());
        }
    }

    /**
     * Stop all processors and pause the camera stream (Level 3).
     */
    public void stop() {
        if (visionPortal == null) {
            throw new IllegalStateException("Vision: VisionPortal is not initialised from init()!");
        }
        // Pause the processor, this will also auto-close any VisionProcessors
        visionPortal.stopStreaming();
        opMode.log("visionportal stopped.");
    }

    /**
     * Get the culmination of data from all attached processors.
     * It is recommended to instead call getData() on individual processors to get their data,
     * however, this method exists to provide a quick way to get all data at once.
     *
     * @return HashMap of all processor data from every attached processor
     */
    @SuppressWarnings({"rawtypes", "unchecked"})
    public HashMap<String, List<VisionData>> getAllData() {
        HashMap<String, List<VisionData>> data = new HashMap<>();
        for (Processor processor : processors) {
            if (Objects.equals(processor.getName(), "rawfeed")) continue;
            data.put(processor.getName(), processor.getData());
        }
        return data;
    }

    /**
     * Terminate all VisionPortal resources (Level 4).
     * <p>
     * Use this method when you are completely done with the VisionPortal and want to free up
     * all available resources. This method will automatically disable all processors and close
     * the VisionPortal, and cannot be undone without calling init() again.
     * <p>
     * It is strongly discouraged to reinitialise the VisionPortal in the same OpMode, as this
     * takes significant time and may cause the OpMode to hang or become unresponsive. Instead,
     * use the {@code start()} and {@code stop()} methods to enable/disable the VisionPortal.
     * Repeated calls to {@code init()} will also cause a termination of the VisionPortal.
     */
    @SuppressWarnings("rawtypes")
    public void terminate() {
        if (visionPortal == null) {
            throw new IllegalStateException("Vision: VisionPortal is not initialised from init()!");
        }
        for (Processor processor : processors) {
            processor.setAttached(false);
        }
        visionPortal.close();
        visionPortal = null;
        opMode.log("visionportal terminated.");
    }

    /**
     * Flip a processor feed horizontally and vertically (rotate 180deg).
     * Should be called after processors are initialised, and can be called at any time after.
     *
     * @param processors Processor instances
     */
    @SuppressWarnings("rawtypes")
    public void flip(Processor... processors) {
        if (visionPortal == null) {
            throw new IllegalStateException("Vision: VisionPortal is not initialised from init()!");
        }
        for (Processor processor : processors) {
            if (processor == null) {
                throw new IllegalStateException("Vision: Processor is not instantiated!");
            }
            if (!this.processors.contains(processor)) {
                throw new IllegalStateException("Vision: Tried to flip a processor that was not initialised!");
            }
            processor.setFlipped(!processor.isFlipped());
            opMode.log("vision processor '%' flipped %.", processor.getName(), processor.isFlipped() ? "upside-down" : "right-side up");
        }
    }

    /**
     * Flip all processor feeds horizontally and vertically (180deg, useful if your camera is mounted upside-down).
     * Should be called after processors are initialised, and can be called at any time after.
     */
    @SuppressWarnings("rawtypes")
    public void flip() {
        if (visionPortal == null) {
            throw new IllegalStateException("Vision: VisionPortal is not initialised from init()!");
        }
        for (Processor processor : processors) {
            processor.setFlipped(!processor.isFlipped());
            opMode.log("vision processor '%' flipped %.", processor.getName(), processor.isFlipped() ? "upside-down" : "right-side up");
        }
    }

    /**
     * Get the current status of the camera attached to the VisionPortal.
     */
    public VisionPortal.CameraState getStatus() {
        if (visionPortal == null) {
            throw new IllegalStateException("Vision: VisionPortal is not initialised from init()!");
        }
        return visionPortal.getCameraState();
    }

    /**
     * Get the current Frames Per Second of the VisionPortal.
     */
    public double getFps() {
        if (visionPortal == null) {
            throw new IllegalStateException("Vision: VisionPortal is not initialised from init()!");
        }
        return visionPortal.getFps();
    }

    /**
     * Returns the state of VisionPortal. Specifically if it is null or not.
     *
     * @return whether the VisionPortal has been initialised with init() or not
     */
    public boolean isInitialised() {
        return visionPortal != null;
    }

    /**
     * Get the VisionPortal directly for advanced operations.
     * This method should be used with caution, as it can be used to directly manipulate the
     * VisionPortal and its resources. It is recommended to use the provided methods in this
     * class to manage the VisionPortal.
     *
     * @return the VisionPortal instance
     */
    public VisionPortal getVisionPortal() {
        if (visionPortal == null) {
            throw new IllegalStateException("Vision: VisionPortal is not initialised from init()!");
        }
        return visionPortal;
    }

    /**
     * Start the VisionSender thread to send all processor data to FtcDashboard.
     */
    public void startDashboardSender() {
        visionSender = new SwitchableVisionSender(this);
        Threads.start(visionSender);
    }

    /**
     * Set the processor to display on FtcDashboard.
     *
     * @param processorName the name of the processor to display on FtcDashboard
     */
    public void setDashboardProcessor(String processorName) {
        if (visionSender != null) {
            visionSender.setStreamingProcessor(processorName);
        }
    }

    /**
     * Set the processor to display on FtcDashboard.
     *
     * @param processor the processor to display on FtcDashboard
     */
    @SuppressWarnings("rawtypes")
    public void setDashboardProcessor(Processor processor) {
        if (visionSender != null) {
            visionSender.setStreamingProcessor(processor.getName());
        }
    }

    /**
     * Stop the VisionSender thread to stop sending all processor data to FtcDashboard.
     * This method is effectively called automatically when the OpMode is no longer active.
     */
    public void stopDashboardSender() {
        if (visionSender != null) {
            Threads.stop(visionSender);
        }
    }

    /**
     * Raw feed processor. Will stream an unprocessed feed.
     * To use this, pass raw as a processor.
     */
    private static class Raw extends Processor<VisionData> {
        @Override
        public String getName() {
            return "raw";
        }

        @Override
        public void update() {
            // no-op
        }

        @Override
        public Object onProcessFrame(Mat frame, long captureTimeNanos) {
            return frame;
        }

        @Override
        public void onFrameDraw(Canvas canvas) {
            // no-op
        }
    }
}