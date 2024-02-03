package org.murraybridgebunyips.bunyipslib.vision;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

import org.murraybridgebunyips.bunyipslib.BunyipsOpMode;
import org.murraybridgebunyips.bunyipslib.BunyipsSubsystem;
import org.murraybridgebunyips.bunyipslib.Dbg;

import java.util.ArrayList;
import java.util.Arrays;

/**
 * Utility component to switch between different feeds and processors with FtcDashboard.
 * The Driver Station usually culminates in a single feed preview, but FtcDashboard
 * requires a bitmap to be sent to it, and it is not very helpful to see the same feed when
 * debugging across different processors.
 *
 * @author Lucas Bubner, 2024
 */
@SuppressWarnings("rawtypes")
@Config
public class SwitchableVisionSender extends BunyipsSubsystem {
    // Can be changed via FtcDashboard
    public static String CURRENT_PROCESSOR_NAME;
    public static int MAX_FPS;
    private final ArrayList<Processor> processors = new ArrayList<>();
    private String lastProcessorName;

    public SwitchableVisionSender(@NonNull BunyipsOpMode opMode, Processor... processors) {
        super(opMode);
        FtcDashboard.getInstance().stopCameraStream();
        this.processors.addAll(Arrays.asList(processors));
    }

    /**
     * Programmatically set the processor to send to FtcDashboard
     *
     * @param processorName the name of the processor to send to FtcDashboard
     */
    public void setStreamingProcessor(String processorName) {
        CURRENT_PROCESSOR_NAME = processorName;
        // Will force an update as we shouldn't need to wait for the activeLoop to do this
        update();
    }

    @Override
    public void update() {
        if (CURRENT_PROCESSOR_NAME == null || CURRENT_PROCESSOR_NAME.equals(lastProcessorName))
            return;

        Processor currentProcessor = processors.stream()
                .filter(p -> p.getName().equals(CURRENT_PROCESSOR_NAME))
                .findFirst()
                .orElse(null);

        if (currentProcessor == null || !currentProcessor.isAttached()) {
            Dbg.error(getClass(), "Unable to find a processor '%' to attached to a Vision system, FtcDashboard sending cancelled.", CURRENT_PROCESSOR_NAME);
            FtcDashboard.getInstance().stopCameraStream();
            return;
        }

        FtcDashboard.getInstance().startCameraStream(currentProcessor, MAX_FPS);
        lastProcessorName = CURRENT_PROCESSOR_NAME;
    }
}
