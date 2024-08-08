package org.murraybridgebunyips.bunyipslib.vision;


import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.stream.CameraStreamServer;
import org.murraybridgebunyips.bunyipslib.Dbg;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

/**
 * Utility component to switch between different feeds and processors with FtcDashboard and the DS "Camera Stream".
 * The Driver Station usually culminates all processors into a single feed preview, and it is not very helpful
 * to see the same feed when debugging across different processors. All processors in BunyipsLib
 * automatically make their own feeds, and we can use this to send different processors as previews.
 * This sender will also try and set the DS feed to the same processor as FtcDashboard, but it is
 * more efficient to use the FtcDashboard feed for debugging -- if available.
 * <p>
 * This sender is not a traditional subsystem, as it is designed to run on another thread not a part of the main loop,
 * similar to how Vision is handled. When started, SwitchableVisionSender will automatically
 * manage the FtcDashboard/DS feed and processor switching, and should be interrupted automatically if used
 * with the Threads utility at the end of a BunyipsOpMode. Ensure to manage your threads properly if
 * not using the Threads utility/methods attached to Vision.
 *
 * @author Lucas Bubner, 2024
 */
@SuppressWarnings("rawtypes")
@Config
public class SwitchableVisionSender implements Runnable {
    /**
     * Can be changed dynamically via FtcDashboard. This is the processor feed
     * that will be sent to FtcDashboard and to the Driver Station feed.
     */
    public static String CURRENT_PROCESSOR_NAME = "";
    /**
     * Can be changed dynamically via FtcDashboard. This is the current Vision instance index
     * that will be used for the {@link #CURRENT_PROCESSOR_NAME}. This index is represented by the array of vision instances
     * that have called {@link Vision#startPreview()} (or have been added to this sender manually).
     */
    public static int CURRENT_PREVIEWING_VISION_INSTANCE_INDEX = 0;
    /**
     * The maximum FPS to send to FtcDashboard.
     */
    public static int MAX_FPS;
    private final ArrayList<Vision> instances = new ArrayList<>();

    private String lastProcessorName;
    private int lastInstanceIdx = -1;

    /**
     * Create a new SwitchableVisionSender.
     *
     * @param firstInstance The base (index 0) vision instance to use with this sender. Additional vision instances
     *                      can be added through {@link #addInstance}.
     */
    public SwitchableVisionSender(@NonNull Vision firstInstance) {
        instances.add(firstInstance);

        // CameraStreamServer will be supplying a raw feed to the DS without this thread
        FtcDashboard.getInstance().stopCameraStream();

        List<Processor> processors = firstInstance.getAttachedProcessors();
        if (processors.isEmpty())
            return;

        // If there is already a current processor name, we should check to see if it is valid
        if (!Objects.equals(CURRENT_PROCESSOR_NAME, "")) {
            for (Processor processor : processors) {
                if (processor.toString().equals(CURRENT_PROCESSOR_NAME)) {
                    // Early return if the processor is valid
                    return;
                }
            }
        }

        CURRENT_PROCESSOR_NAME = firstInstance.getAttachedProcessors().get(0).toString();
    }

    /**
     * Add an instance that can later be accessed dynamically for selection through {@link #CURRENT_PREVIEWING_VISION_INSTANCE_INDEX}.
     *
     * @param instance The instance to add.
     * @return The index that can be used to set the previewing instance index.
     */
    public int addInstance(@NonNull Vision instance) {
        instances.add(instance);
        return instances.size() - 1;
    }

    /**
     * Programmatically set the processor to send to FtcDashboard
     *
     * @param processorName the name of the processor to send to FtcDashboard
     */
    public void setStreamingProcessor(String processorName) {
        CURRENT_PROCESSOR_NAME = processorName;
    }

    /**
     * Set the instance to use for vision previews.
     *
     * @param instance the vision instance, if it has already been added via {@link #addInstance}.
     */
    public void setInstance(@NonNull Vision instance) {
        if (!instances.contains(instance))
            throw new IllegalArgumentException("Instances does not contain this instance! It must be added prior through addInstance()");
        CURRENT_PREVIEWING_VISION_INSTANCE_INDEX = instances.indexOf(instance);
    }

    /**
     * Set the index to use for vision previews.
     *
     * @param index the index to use and to send to FtcDashboard
     */
    public void setInstance(int index) {
        CURRENT_PREVIEWING_VISION_INSTANCE_INDEX = index;
    }

    @Override
    public void run() {
        while (!Thread.currentThread().isInterrupted()) {
            if ((Objects.equals(CURRENT_PROCESSOR_NAME, "") || CURRENT_PROCESSOR_NAME.equals(lastProcessorName))
                    && (CURRENT_PREVIEWING_VISION_INSTANCE_INDEX == lastInstanceIdx))
                continue;

            if (CURRENT_PREVIEWING_VISION_INSTANCE_INDEX < 0 || CURRENT_PREVIEWING_VISION_INSTANCE_INDEX >= instances.size()) {
                Dbg.error(getClass(), "Current vision index is out of bounds, FtcDashboard sending cancelled.");
                FtcDashboard.getInstance().stopCameraStream();
                CameraStreamServer.getInstance().setSource(null);
                continue;
            }

            Processor currentProcessor = instances.get(CURRENT_PREVIEWING_VISION_INSTANCE_INDEX).getAttachedProcessors().stream()
                    .filter(p -> p.toString().equals(CURRENT_PROCESSOR_NAME))
                    .findFirst()
                    .orElse(null);

            Dbg.logd(getClass(), "Preview processor updated (instance #%->#%), %->%", lastInstanceIdx, CURRENT_PREVIEWING_VISION_INSTANCE_INDEX, lastProcessorName, currentProcessor);
            lastInstanceIdx = CURRENT_PREVIEWING_VISION_INSTANCE_INDEX;
            lastProcessorName = CURRENT_PROCESSOR_NAME;

            if (currentProcessor == null) {
                Dbg.error(getClass(), "Unable to find a processor '%' to attached to the #% Vision instance, FtcDashboard sending cancelled.", CURRENT_PROCESSOR_NAME, CURRENT_PREVIEWING_VISION_INSTANCE_INDEX);
                FtcDashboard.getInstance().stopCameraStream();
                CameraStreamServer.getInstance().setSource(null);
                continue;
            }

            FtcDashboard.getInstance().startCameraStream(currentProcessor, MAX_FPS);
            CameraStreamServer.getInstance().setSource(currentProcessor);
        }
        FtcDashboard.getInstance().stopCameraStream();
        CameraStreamServer.getInstance().setSource(null);
    }
}
