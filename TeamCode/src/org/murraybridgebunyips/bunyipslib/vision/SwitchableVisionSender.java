package org.murraybridgebunyips.bunyipslib.vision;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

import org.murraybridgebunyips.bunyipslib.Dbg;

import java.util.List;
import java.util.Objects;

/**
 * Utility component to switch between different feeds and processors with FtcDashboard.
 * The Driver Station usually culminates all processors into  a single feed preview, and it is not very helpful
 * to see the same feed when debugging across different processors. FtcDashboard allows us to send
 * custom bitmaps, and we can use this to send the feed from different processors to the dashboard.
 * <p>
 * This sender is not a traditional subsystem, as it is designed to run on another thread not a part of the main loop,
 * similar to how Vision is handled. When started, SwitchableVisionSender will automatically
 * manage the FtcDashboard feed and processor switching, and should be interrupted automatically if used
 * with the Threads utility at the end of a BunyipsOpMode. Ensure to manage your threads properly if
 * not using the Threads utility.
 *
 * @author Lucas Bubner, 2024
 */
@SuppressWarnings("rawtypes")
@Config
public class SwitchableVisionSender implements Runnable {
    // Can be changed via FtcDashboard
    public static String CURRENT_PROCESSOR_NAME = "";
    public static int MAX_FPS;
    private final Vision vision;
    private String lastProcessorName;

    public SwitchableVisionSender(Vision vision) {
        FtcDashboard.getInstance().stopCameraStream();
        this.vision = vision;

        List<Processor> processors = vision.getAttachedProcessors();
        if (processors.size() == 0)
            return;

        // If there is already a current processor name, we should check to see if it is valid
        if (!Objects.equals(CURRENT_PROCESSOR_NAME, "")) {
            for (Processor processor : processors) {
                if (processor.getName().equals(CURRENT_PROCESSOR_NAME)) {
                    // Early return if the processor is valid
                    return;
                }
            }
        }

        CURRENT_PROCESSOR_NAME = vision.getAttachedProcessors().get(0).getName();
    }

    /**
     * Programmatically set the processor to send to FtcDashboard
     *
     * @param processorName the name of the processor to send to FtcDashboard
     */
    public void setStreamingProcessor(String processorName) {
        CURRENT_PROCESSOR_NAME = processorName;
    }

    @Override
    public void run() {
        while (!Thread.currentThread().isInterrupted()) {
            if (Objects.equals(CURRENT_PROCESSOR_NAME, "") || CURRENT_PROCESSOR_NAME.equals(lastProcessorName))
                continue;

            Processor currentProcessor = vision.getAttachedProcessors().stream()
                    .filter(p -> p.getName().equals(CURRENT_PROCESSOR_NAME))
                    .findFirst()
                    .orElse(null);

            lastProcessorName = CURRENT_PROCESSOR_NAME;

            if (currentProcessor == null) {
                Dbg.error(getClass(), "Unable to find a processor '%' to attached to a Vision system, FtcDashboard sending cancelled.", CURRENT_PROCESSOR_NAME);
                FtcDashboard.getInstance().stopCameraStream();
                continue;
            }

            FtcDashboard.getInstance().startCameraStream(currentProcessor, MAX_FPS);
        }
    }
}
