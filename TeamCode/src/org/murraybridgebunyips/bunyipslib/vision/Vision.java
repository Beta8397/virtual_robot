package org.murraybridgebunyips.bunyipslib.vision;


import static org.murraybridgebunyips.bunyipslib.Text.round;

import android.graphics.Canvas;
import android.util.Size;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.murraybridgebunyips.bunyipslib.BunyipsSubsystem;
import org.murraybridgebunyips.bunyipslib.Dbg;
import org.murraybridgebunyips.bunyipslib.Threads;
import org.murraybridgebunyips.bunyipslib.vision.data.VisionData;
import org.opencv.core.Mat;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Objects;
import java.util.stream.Collectors;

/**
 * Component wrapper to support the v8.2+ SDK's included libraries for Camera operation.
 * This is an expansible system to run Processor components using the VisionPortal.
 * <p>
 * You will pass your own processors that you manage, and Vision will handle the data collection.
 * <p>
 * Vision is not a traditional subsystem where updates are propagated on the main thread,
 * as processing is on another thread and updates are managed at the discretion of the VisionPortal.
 * Once set up, Vision will automatically manage the camera stream and defined processor updates.
 * All you will need to do is collect the data from the processors and use it in your OpMode. The
 * update() method in this subsystem will simply add telemetry of the VisionPortal's status.
 *
 * @author Lucas Bubner, 2023
 */
@Config
public class Vision extends BunyipsSubsystem {
    public static int CAMERA_WIDTH = 640;
    public static int CAMERA_HEIGHT = 480;
    // Static: only one sender can be active at a time
    private static SwitchableVisionSender visionSender;
    @SuppressWarnings("rawtypes")
    private final List<Processor> processors = new ArrayList<>();
    private final CameraName camera;
    /**
     * A built-in raw feed Processor that will do nothing but provide the raw camera feed.
     * Useful for debugging and testing, pass this raw field (vision.raw) to init() and start() to use it.
     */
    public Raw raw = new Raw();
    private VisionPortal visionPortal;

    public Vision(CameraName camera, int cameraWidth, int cameraHeight) {
        this.camera = camera;
        // Allow the user to set the camera resolution if they want
        CAMERA_WIDTH = cameraWidth;
        CAMERA_HEIGHT = cameraHeight;
        // Vision itself is generally never used with tasks, so we can mute the scheduler reports
        muteTaskReports();
    }

    public Vision(CameraName camera) {
        this(camera, CAMERA_WIDTH, CAMERA_HEIGHT);
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
            Dbg.logd(getClass(), "WARNING: Vision already initialised! Tearing down...");
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
            Dbg.logd(getClass(), "vision processor '%' initialised.", processor.getName());
        }

        // Since Vision is usually called from the init-cycle, we can try to fit in some telemetry
        opMode.addTelemetry(
                "Vision: % processor(s) initialised.",
                Arrays.stream(processors).map(Processor::getName).collect(Collectors.toList())
        );

        visionPortal = builder
                .setCamera(camera)
                .setCameraResolution(new Size(CAMERA_WIDTH, CAMERA_HEIGHT))
                // "Live View" does not affect how the DS/Dashboard stream is handled, as these previews
                // are not connected to the Live View. As such, to save resources, it is
                // better we leave the live view off, as this will not reflect the DS/Dashboard stream.
                // As for the DS Camera Stream feature and FtcDashboard, you will need to look at
                // the startPreview() method to enable the VisionSender. By default, the DS Camera Stream
                // will show a raw feed from the camera, and the FtcDashboard feed will be disabled.
                .enableLiveView(false)
                // Set any additional VisionPortal settings here
                .build();

        // Disable the vision processors by default. The OpMode must call start() to enable them.
        for (Processor processor : this.processors) {
            visionPortal.setProcessorEnabled(processor, false);
        }

        Dbg.logd(getClass(), "visionportal ready.");
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
            Dbg.logd(getClass(), "visionportal restarting...");
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
            Dbg.logd(getClass(), "vision processor '%' started.", processor.getName());
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
            Dbg.logd(getClass(), "vision processor '%' paused.", processor.getName());
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
        Dbg.logd(getClass(), "visionportal stopped.");
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
        Dbg.logd(getClass(), "visionportal terminated.");
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
            Dbg.logd(getClass(), "vision processor '%' flipped %.", processor.getName(), processor.isFlipped() ? "upside-down" : "right-side up");
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
            Dbg.logd(getClass(), "vision processor '%' flipped %.", processor.getName(), processor.isFlipped() ? "upside-down" : "right-side up");
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
     * Without the preview active, the DS will display a raw unprocessed feed to save resources,
     * but activating this sender will set both FtcDashboard and the DS streams to be of a processor
     * of your choosing.
     *
     * @see SwitchableVisionSender
     */
    public void startPreview() {
        visionSender = new SwitchableVisionSender(this);
        Threads.start(visionSender);
    }

    /**
     * Set the processor to display on FtcDashboard.
     *
     * @param processorName the name of the processor to display on FtcDashboard
     * @see SwitchableVisionSender
     */
    public void setPreview(String processorName) {
        if (visionSender != null) {
            visionSender.setStreamingProcessor(processorName);
        }
    }

    /**
     * Set the processor to display on FtcDashboard.
     *
     * @param processor the processor to display on FtcDashboard
     * @see SwitchableVisionSender
     */
    @SuppressWarnings("rawtypes")
    public void setPreview(Processor processor) {
        if (visionSender != null) {
            visionSender.setStreamingProcessor(processor.getName());
        }
    }

    /**
     * Stop the VisionSender thread to stop sending all processor data to FtcDashboard.
     * This method is effectively called automatically when the OpMode is no longer active.
     *
     * @see SwitchableVisionSender
     */
    public void stopPreview() {
        if (visionSender != null) {
            Threads.stop(visionSender);
            visionSender = null;
        }
    }

    @Override
    public void update() {
        if (visionPortal != null) {
            opMode.addTelemetry(
                    "Vision: % | % fps | %/% processors",
                    visionPortal.getCameraState(),
                    (int) round(visionPortal.getFps(), 0),
                    processors.stream().filter((p) -> visionPortal.getProcessorEnabled(p)).count(),
                    processors.size()
            );
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
        public void onFrameDraw(Canvas canvas, Object userContext) {
            // no-op
        }
    }
}