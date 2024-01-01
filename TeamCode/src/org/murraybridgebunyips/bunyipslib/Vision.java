package org.murraybridgebunyips.bunyipslib;


import android.util.Size;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.murraybridgebunyips.bunyipslib.vision.Processor;
import org.murraybridgebunyips.bunyipslib.vision.data.VisionData;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;

/**
 * Component wrapper to support the v8.2+ SDK's included libraries for Camera operation.
 * This is an expansible system to run Processor components using the VisionPortal.
 *
 * @author Lucas Bubner, 2023
 */
@Config
public class Vision extends BunyipsComponent {
    public static int CAMERA_WIDTH = 1280;
    public static int CAMERA_HEIGHT = 720;
    @SuppressWarnings("rawtypes")
    private final List<Processor> processors = new ArrayList<>();
    private final CameraName camera;
    private VisionPortal visionPortal;

    public Vision(@NonNull BunyipsOpMode opMode, CameraName camera, int cameraWidth, int cameraHeight) {
        super(opMode);
        this.camera = camera;
        // Allow the user to set the camera resolution if they want
        CAMERA_WIDTH = cameraWidth;
        CAMERA_HEIGHT = cameraHeight;
    }

    public Vision(@NonNull BunyipsOpMode opMode, CameraName camera) {
        super(opMode);
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
            getOpMode().log("WARNING: Vision already initialised! Tearing down...");
            visionPortal.close();
            visionPortal = null;
        }

        if (processors.length == 0) {
            throw new IllegalArgumentException("Vision: Must initialise at least one integrated processor!");
        }

        // Hand over instance control to the VisionPortal
        this.processors.addAll(Arrays.asList(processors));

        // Initialise the VisionPortal with our newly created processors
        VisionPortal.Builder builder = new VisionPortal.Builder();
        for (Processor processor : processors) {
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
            getOpMode().log("vision processor '%' initialised.", processor.getClass().getSimpleName());
        }

        visionPortal = builder
                .setCamera(camera)
                .setCameraResolution(new Size(CAMERA_WIDTH, CAMERA_HEIGHT))
                .enableLiveView(true)
                .setAutoStopLiveView(true)
                // Set any additional VisionPortal settings here
                .build();

        // Disable the vision processors by default. The OpMode must call start() to enable them.
        for (Processor processor : this.processors) {
            visionPortal.setProcessorEnabled(processor, false);
        }

        // Disable live view by default
        visionPortal.stopLiveView();
        getOpMode().log("visionportal ready.");
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
            getOpMode().log("visionportal restarting...");
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
            getOpMode().log("vision processor '%' started.", processor.getClass().getSimpleName());
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
            getOpMode().log("vision processor '%' paused.", processor.getClass().getSimpleName());
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
        getOpMode().log("visionportal stopped.");
    }

    /**
     * Tick all processor camera streams and extract data from the processors.
     * This can optionally be done per processor by calling processor.tick()
     * This data is stored in the processor instance and can be accessed with the getters.
     */
    @SuppressWarnings("rawtypes")
    public void tickAll() {
        for (Processor processor : processors) {
            processor.tick();
        }
    }

    /**
     * Get data from all processors after being ticked.
     * This can optionally can be done per processor by calling processor.getData().
     * This data is stored in the processor instance and can be accessed with getters.
     *
     * @return HashMap of all processor data from every attached processor
     */
    @SuppressWarnings({"rawtypes", "unchecked"})
    public HashMap<String, List<VisionData>> getAllData() {
        HashMap<String, List<VisionData>> data = new HashMap<>();
        for (Processor processor : processors) {
            if (processor.getClass().getSimpleName().equals("NoData")) continue;
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
    public void terminate() {
        if (visionPortal == null) {
            throw new IllegalStateException("Vision: VisionPortal is not initialised from init()!");
        }
        visionPortal.close();
        visionPortal = null;
        getOpMode().log("visionportal terminated.");
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
            getOpMode().log("vision processor '%' flipped %.", processor.getClass().getSimpleName(), processor.isFlipped() ? "upside-down" : "right-side up");
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
            getOpMode().log("vision processor '%' flipped %.", processor.getClass().getSimpleName(), processor.isFlipped() ? "upside-down" : "right-side up");
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
     * Start or stop the live camera view (Level 1).
     * When initialised, live view is disabled by default.
     */
    public void setLiveView(boolean enabled) {
        if (visionPortal == null) {
            throw new IllegalStateException("Vision: VisionPortal is not initialised from init()!");
        }
        if (enabled) {
            visionPortal.resumeLiveView();
        } else {
            visionPortal.stopLiveView();
        }
    }

    /**
     * Returns the state of VisionPortal. Specifically if it is null or not.
     *
     * @return whether the VisionPortal has been initialised with init() or not
     */
    public boolean isInitialised() {
        return visionPortal != null;
    }
}