package org.murraybridgebunyips.bunyipslib.vision.processors.centerstage;

import android.graphics.Canvas;

import com.acmerobotics.dashboard.config.Config;

import org.murraybridgebunyips.bunyipslib.vision.Processor;
import org.murraybridgebunyips.bunyipslib.vision.Vision;
import org.murraybridgebunyips.bunyipslib.vision.data.TeamPropData;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;

import java.util.Arrays;
import java.util.List;

/**
 * Detection for a custom team prop based on colour ranges,
 * refactored to work with our vision system
 *
 * @author FTC 14133, <a href="https://github.com/FTC14133/FTC14133-2023-2024/blob/Detection-TeamElement/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Subsystems/TeamElementDetection/Pipeline/SplitAveragePipeline.java">...</a>
 * @noinspection FieldCanBeLocal
 */
@Config
public class TeamProp extends Processor<TeamPropData> {
    public static double NONE_THRESHOLD = 220;
    private final List<Integer> ELEMENT_COLOR;

    private double distance1;
    private double distance2;
    private double max_distance;

    private Mat zone1;
    private Mat zone2;
    private Scalar avgColor1;
    private Scalar avgColor2;

    /**
     * @param r Red value of the element color (0-255)
     * @param g Green value of the element color (0-255)
     * @param b Blue value of the element color (0-255)
     */
    public TeamProp(int r, int g, int b) {
        ELEMENT_COLOR = Arrays.asList(r, g, b);
    }

    @Override
    public Object onProcessFrame(Mat frame, long captureTimeNanos) {
        zone1 = frame.submat(new Rect(0, 0, Vision.CAMERA_WIDTH / 2, Vision.CAMERA_HEIGHT));
        zone2 = frame.submat(new Rect(Vision.CAMERA_WIDTH / 2, 0, Vision.CAMERA_WIDTH / 2, Vision.CAMERA_HEIGHT));

        avgColor1 = Core.mean(zone1);
        avgColor2 = Core.mean(zone2);
        zone1.setTo(avgColor1);
        zone2.setTo(avgColor2);

        distance1 = color_distance(avgColor1, ELEMENT_COLOR);
        distance2 = color_distance(avgColor2, ELEMENT_COLOR);

        if (distance1 > NONE_THRESHOLD && distance2 > NONE_THRESHOLD) {
            max_distance = -1;
        } else {
            // Don't ask
            max_distance = Math.min(distance1, distance2);
        }

        return frame;
    }

    @SuppressWarnings("rawtypes")
    public double color_distance(Scalar color1, List color2) {
        double r1 = color1.val[0];
        double g1 = color1.val[1];
        double b1 = color1.val[2];

        int r2 = (int) color2.get(0);
        int g2 = (int) color2.get(1);
        int b2 = (int) color2.get(2);

        return Math.sqrt(Math.pow((r1 - r2), 2) + Math.pow((g1 - g2), 2) + Math.pow((b1 - b2), 2));
    }


    @Override
    public void onFrameDraw(Canvas canvas) {
        // no-op
    }

    @Override
    public String getName() {
        return "teamprop";
    }

    @Override
    public void update() {
        if (max_distance == -1) {
            data.add(new TeamPropData(Positions.RIGHT, distance1, distance2, max_distance));
            return;
        }
        if (max_distance == distance1) {
            data.add(new TeamPropData(Positions.LEFT, distance1, distance2, max_distance));
            return;
        }
        data.add(new TeamPropData(Positions.CENTER, distance1, distance2, max_distance));

        zone1.release();
        zone2.release();
    }

    public enum Positions {
        LEFT,
        CENTER,
        RIGHT
    }
}