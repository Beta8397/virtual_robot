package org.murraybridgebunyips.bunyipslib.vision.processors;

import android.graphics.Canvas;
import android.graphics.Paint;
import android.util.Pair;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;

import org.murraybridgebunyips.bunyipslib.vision.Processor;
import org.murraybridgebunyips.bunyipslib.vision.Vision;
import org.murraybridgebunyips.bunyipslib.vision.data.ContourData;
import org.opencv.core.Mat;

import java.util.ArrayList;

/**
 * A processor that applies multiple ColourThreshold processors to the same frame and
 * combines their draw results. The MASK variable can be used to select which processor's mask
 * is drawn to the frame.
 *
 * @author Lucas Bubner, 2024
 * @see ColourThreshold
 */
@Config
public class MultiColourThreshold extends Processor<ContourData> {
    /**
     * The index of the processor mask to draw to the frame.
     */
    public static int MASK;
    private final ArrayList<Pair<ColourThreshold, Mat>> colourProcessors = new ArrayList<>();

    /**
     * Create a new MultiColourThreshold with the given processors.
     *
     * @param thresholdProcessors the colour processors to use
     */
    public MultiColourThreshold(ColourThreshold... thresholdProcessors) {
        for (ColourThreshold processor : thresholdProcessors) {
            colourProcessors.add(new Pair<>(processor, new Mat()));
        }
    }

    @NonNull
    @Override
    public String toString() {
        return "multicolourthreshold";
    }

    @Override
    public void update() {
        for (Pair<ColourThreshold, Mat> processor : colourProcessors) {
            processor.first.clearData();
            processor.first.update();
            data.addAll(processor.first.getData());
            processor.second.release();
        }
    }

    @Override
    public void onProcessFrame(Mat frame, long captureTimeNanos) {
        for (Pair<ColourThreshold, Mat> processor : colourProcessors) {
            frame.copyTo(processor.second);
            processor.first.onProcessFrame(processor.second, captureTimeNanos);
        }
        if (!colourProcessors.isEmpty() && MASK >= 1 && MASK <= colourProcessors.size()) {
            colourProcessors.get(MASK - 1).second.copyTo(frame);
        }
    }

    @Override
    public void onFrameDraw(Canvas canvas) {
        for (Pair<ColourThreshold, Mat> processor : colourProcessors) {
            processor.first.onFrameDraw(canvas);
        }
        // Display mask name on camera feed
        canvas.drawText(
                MASK >= 1 && MASK <= colourProcessors.size() ? colourProcessors.get(MASK - 1).first.toString() : "",
                10,
                Vision.CAMERA_HEIGHT - 10,
                new Paint() {{
                    setColor(0xFFFFFFFF);
                    setStrokeWidth(30);
                    setTextSize(20);
                    setAntiAlias(true);
                }}
        );
    }
}
