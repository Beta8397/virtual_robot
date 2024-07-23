package org.murraybridgebunyips.bunyipslib.vision;


import static org.murraybridgebunyips.bunyipslib.Text.round;

import android.graphics.Canvas;
import android.util.Size;

import androidx.annotation.NonNull;

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
    /**
     * Default camera width resolution to use.
     */
    public static int DEFAULT_CAMERA_WIDTH = 640;
    /**
     * Default camera height resolution to use.
     */
    public static int DEFAULT_CAMERA_HEIGHT = 480;
    // Singleton: Additional instances are managed internally by SwitchableVisionSender
    private static SwitchableVisionSender visionSender = null;
    private final Size preferredResolution;
    @SuppressWarnings("rawtypes")
    private final List<Processor> processors = new ArrayList<>();
    private final CameraName camera;
    /**
     * A built-in raw feed Processor that will do nothing but provide the raw camera feed.
     * Useful for debugging and testing, pass this raw field (vision.raw) to init() and start() to use it.
     *
     * @noinspection ClassEscapesDefinedScope (should never be used outside of this class, and checks exist for usage in other classes)
     */
    public Raw raw = new Raw();
    private int senderIdx = -1;
    private VisionPortal visionPortal = null;

    /**
     * Create a new Vision instance with the specified camera and resolution.
     *
     * @param camera       The camera to use
     * @param cameraWidth  The camera width resolution to use
     * @param cameraHeight The camera height resolution to use
     */
    public Vision(CameraName camera, int cameraWidth, int cameraHeight) {
        assertParamsNotNull(camera);
        this.camera = camera;
        // Allow the user to set the camera resolution if they want
        preferredResolution = new Size(cameraWidth, cameraHeight);
    }

    /**
     * Create a new Vision instance with the specified camera and default resolution.
     *
     * @param camera The camera to use
     */
    public Vision(CameraName camera) {
        this(camera, DEFAULT_CAMERA_WIDTH, DEFAULT_CAMERA_HEIGHT);
    }

    /**
     * Stop the VisionSender thread to stop sending all processor data to FtcDashboard/DS.
     * The DS will return to a raw unprocessed feed, and FtcDashboard feed will be disabled.
     * This method is effectively called automatically when the OpMode is no longer active.
     * <p>
     * If you don't need to use a preview from a particular camera anymore, consider switching the index away
     * from the camera to one that you still need a preview for, else call this method to free unused resources.
     *
     * @see SwitchableVisionSender
     */
    public static void stopAllPreviews() {
        if (visionSender != null) {
            // This method will be automatically called as part of the BunyipsOpMode cleanup process
            Threads.stop(visionSender);
            visionSender = null;
        }
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
     * @param newProcessors Processor instances
     * @return the vision instance
     */
    @SuppressWarnings("rawtypes")
    public Vision init(Processor... newProcessors) {
        if (visionPortal != null) {
            Dbg.warn(getClass(), "%visionportal instance already initialised! tearing down...", isDefaultName() ? "" : "(" + name + ") ");
            terminate();
        }

        if (newProcessors.length == 0) {
            throw new IllegalArgumentException("Vision: Must initialise at least one integrated processor!");
        }

        // Hand over instance control to the VisionPortal
        processors.addAll(Arrays.asList(newProcessors));

        // Initialise the VisionPortal with our newly created processors
        VisionPortal.Builder builder = new VisionPortal.Builder();
        for (Processor processor : processors) {
            if (processor == null) {
                throw new IllegalStateException("Vision: Processor is not instantiated!");
            }
            if (processor.toString().isEmpty()) {
                throw new IllegalStateException("Vision: Processor name cannot be empty!");
            }
            if (processor instanceof Raw && processor != raw) {
                throw new IllegalStateException("Vision: Processor is using a Raw instance from a different vision instance!");
            }
            for (Processor otherProcessor : processors) {
                if (otherProcessor != processor && otherProcessor.toString().equals(processor.toString())) {
                    throw new IllegalStateException("Vision: Processor name must be unique!");
                }
            }
            // An exception will be thrown if this processor is already attached to another Vision instance
            builder.addProcessor(processor);
            processor.attach(preferredResolution);
            Dbg.logv(getClass(), "%vision processor '%' initialised.", isDefaultName() ? "" : "(" + name + ") ", processor.toString());
        }

        // Since Vision is usually called from the init-cycle, we can try to fit in some telemetry
        opMode.telemetry.add(
                "%: % processor(s) initialised.",
                name,
                Arrays.stream(newProcessors).map(Processor::toString).collect(Collectors.toList())
        );

        visionPortal = builder
                .setCamera(camera)
                .setCameraResolution(new Size(DEFAULT_CAMERA_WIDTH, DEFAULT_CAMERA_HEIGHT))
                // "Live View" does not affect how the DS/Dashboard stream is handled, as these previews
                // are not connected to the Live View. As such, to save resources, it is
                // better we leave the live view off, as this will not reflect the DS/Dashboard stream.
                // As for the DS Camera Stream feature and FtcDashboard, you will need to look at
                // the startPreview() method to enable the VisionSender. By default, the DS Camera Stream
                // will show a raw feed from the camera, and the FtcDashboard feed will be disabled.
                .enableLiveView(false)
                .setShowStatsOverlay(false)
                // Set any additional VisionPortal settings here
                .build();

        visionPortal.stopLiveView();

        // Disable the vision processors by default. The OpMode must call start() to enable them.
        for (Processor processor : processors) {
            visionPortal.setProcessorEnabled(processor, false);
        }

        Dbg.logv(getClass(), "%visionportal ready.", isDefaultName() ? "" : "(" + name + ") ");
        return this;
    }

    /**
     * Start desired processors. This method must be called before trying to extract data from
     * the cameras, and must be already initialised with the init() method.
     *
     * @param attachedProcessors Processor instances
     * @return the vision instance
     */
    @SuppressWarnings("rawtypes")
    public Vision start(Processor... attachedProcessors) {
        if (visionPortal == null) {
            throw new IllegalStateException("Vision: VisionPortal is not initialised from init()!");
        }

        // Resume the stream if it was previously stopped or is not running
        if (visionPortal.getCameraState() == VisionPortal.CameraState.CAMERA_DEVICE_READY ||
                visionPortal.getCameraState() == VisionPortal.CameraState.STOPPING_STREAM) {
            // Note if the camera state is STOPPING_STREAM, it will block the thread until the
            // stream is resumed. This is a documented operation in the SDK.
            Dbg.logv(getClass(), "%visionportal restarting...", isDefaultName() ? "" : "(" + name + ") ");
            visionPortal.resumeStreaming();
        }

        for (Processor processor : attachedProcessors) {
            if (processor == null) {
                throw new IllegalStateException("Vision: Processor is not instantiated!");
            }
            if (!processors.contains(processor)) {
                throw new IllegalStateException("Vision: Tried to start a processor that was not initialised!");
            }
            visionPortal.setProcessorEnabled(processor, true);
            processor.setRunning(true);
            Dbg.logv(getClass(), "%vision processor '%' started.", isDefaultName() ? "" : "(" + name + ") ", processor.toString());
        }
        return this;
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
     * @param attachedProcessors Processor instances
     * @return the vision instance
     */
    @SuppressWarnings("rawtypes")
    public Vision stop(Processor... attachedProcessors) {
        if (visionPortal == null) {
            throw new IllegalStateException("Vision: VisionPortal is not initialised from init()!");
        }

        // Disable processors without pausing the stream
        for (Processor processor : attachedProcessors) {
            if (processor == null) {
                throw new IllegalStateException("Vision: Processor is not instantiated!");
            }
            if (!processors.contains(processor)) {
                throw new IllegalStateException("Vision: Tried to stop a processor that was not initialised!");
            }
            visionPortal.setProcessorEnabled(processor, false);
            processor.setRunning(false);
            Dbg.logv(getClass(), "%vision processor '%' paused.", isDefaultName() ? "" : "(" + name + ") ", processor.toString());
        }
        return this;
    }

    /**
     * Stop all processors and pause the camera stream (Level 3).
     *
     * @return the vision instance
     */
    public Vision stop() {
        if (visionPortal == null) {
            throw new IllegalStateException("Vision: VisionPortal is not initialised from init()!");
        }
        // Pause the VisionPortal, this will stagnate any attached processors
        visionPortal.stopStreaming();
        Dbg.logv(getClass(), "%visionportal stopped.", isDefaultName() ? "" : "(" + name + ") ");
        return this;
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
            if (Objects.equals(processor.toString(), "raw")) continue;
            data.put(processor.toString(), processor.getData());
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
     *
     * @return the vision instance
     */
    @SuppressWarnings("rawtypes")
    public Vision terminate() {
        if (visionPortal == null) {
            throw new IllegalStateException("Vision: VisionPortal is not initialised from init()!");
        }
        for (Processor processor : processors) {
            processor.detach();
        }
        visionPortal.close();
        visionPortal = null;
        Dbg.logv(getClass(), "%visionportal terminated.", isDefaultName() ? "" : "(" + name + ") ");
        return this;
    }

    /**
     * Flip a processor feed horizontally and vertically (rotate 180deg).
     * Should be called after processors are initialised, and can be called at any time after.
     *
     * @param attachedProcessors Processor instances
     * @return the vision instance
     */
    @SuppressWarnings("rawtypes")
    public Vision flip(Processor... attachedProcessors) {
        if (visionPortal == null) {
            throw new IllegalStateException("Vision: VisionPortal is not initialised from init()!");
        }
        for (Processor processor : attachedProcessors) {
            if (processor == null) {
                throw new IllegalStateException("Vision: Processor is not instantiated!");
            }
            if (!processors.contains(processor)) {
                throw new IllegalStateException("Vision: Tried to flip a processor that was not initialised!");
            }
            processor.setFlipped(!processor.isFlipped());
            Dbg.logv(getClass(), "%vision processor '%' flipped %.", isDefaultName() ? "" : "(" + name + ") ", processor.toString(), processor.isFlipped() ? "upside-down" : "right-side up");
        }
        return this;
    }

    /**
     * Flip all processor feeds horizontally and vertically (180deg, useful if your camera is mounted upside-down).
     * Should be called after processors are initialised, and can be called at any time after.
     *
     * @return the vision instance
     */
    @SuppressWarnings("rawtypes")
    public Vision flip() {
        if (visionPortal == null) {
            throw new IllegalStateException("Vision: VisionPortal is not initialised from init()!");
        }
        for (Processor processor : processors) {
            processor.setFlipped(!processor.isFlipped());
            Dbg.logv(getClass(), "%vision processor '%' flipped %.", isDefaultName() ? "" : "(" + name + ") ", processor.toString(), processor.isFlipped() ? "upside-down" : "right-side up");
        }
        return this;
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
     * Start the VisionSender thread to send all processor data to FtcDashboard/DS with dynamic switching.
     * Without the preview active, the DS will display a raw unprocessed feed to save resources in
     * Camera Stream, and FtcDashboard will be disabled. This is the default state of Vision.
     * Activating this sender will set both FtcDashboard and the DS streams to be of a processor
     * of your choosing, by changing a processor's name with setPreview() or via FtcDashboard.
     *
     * @see SwitchableVisionSender
     */
    public void startPreview() {
        if (visionSender == null || !Threads.isRunning(visionSender)) {
            visionSender = new SwitchableVisionSender(this);
            Threads.start(visionSender);
            senderIdx = 0;
        } else {
            senderIdx = visionSender.addInstance(this);
        }
    }

    /**
     * Set the processor to display on FtcDashboard/DS from startPreview().
     * This will automatically switch the instance index, allowing an instant stream of this camera and processor combo.
     * Will no-op if this processor name is invalid.
     *
     * @param processorName the name of the processor to display on FtcDashboard
     * @see SwitchableVisionSender
     */
    public void setPreview(String processorName) {
        if (senderIdx != -1 && visionSender != null && processors.stream().anyMatch((p) -> processorName.equals(p.toString()))) {
            visionSender.setInstance(senderIdx);
            visionSender.setStreamingProcessor(processorName);
        }
    }

    /**
     * Set the processor to display on FtcDashboard/DS from startPreview().
     * This will automatically switch the instance index, allowing an instant stream of this camera and processor combo.
     * Will no-op if this processor is invalid.
     *
     * @param processor the processor to display on FtcDashboard
     * @see SwitchableVisionSender
     */
    @SuppressWarnings("rawtypes")
    public void setPreview(Processor processor) {
        setPreview(processor.toString());
    }

    /**
     * Optional telemetry for subsystem attachment
     */
    @Override
    protected void periodic() {
        if (visionPortal != null) {
            VisionPortal.CameraState state = visionPortal.getCameraState();
            opMode.telemetry.add(
                    "%: <font color='%'>%</font> | % fps | %/% processors",
                    name,
                    state == VisionPortal.CameraState.STREAMING ? "green" : state == VisionPortal.CameraState.ERROR ? "red" : "yellow",
                    state,
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
        @NonNull
        @Override
        public String toString() {
            return "raw";
        }

        @Override
        public void update() {
            // no-op
        }

        @Override
        public void onProcessFrame(Mat frame, long captureTimeNanos) {
            // no-op
        }

        @Override
        public void onFrameDraw(Canvas canvas) {
            // no-op
        }
    }
}