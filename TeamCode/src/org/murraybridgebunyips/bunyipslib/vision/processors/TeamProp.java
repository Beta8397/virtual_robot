package org.murraybridgebunyips.bunyipslib.vision.processors;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
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
public class TeamProp extends Processor<TeamPropData> {
    private final List<Integer> ELEMENT_COLOR;
    private final int line1x = Vision.CAMERA_WIDTH / 3;
    private final int line2x = (Vision.CAMERA_WIDTH / 3) * 2;

    private double distance1;
    private double distance2;
    private double distance3;
    private double max_distance;

    // Patch to avoid leaking memory
    private Mat zone1;
    private Mat zone2;
    private Mat zone3;
    private Scalar avgColor1;
    private Scalar avgColor2;
    private Scalar avgColor3;

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
        // Rect (top left x, top left y, bottom right x, bottom right y)
        zone1 = frame.submat(new Rect(0, 0, line1x, Vision.CAMERA_HEIGHT));
        zone2 = frame.submat(new Rect(line1x, 0, line2x - line1x, Vision.CAMERA_HEIGHT));
        zone3 = frame.submat(new Rect(line2x, 0, Vision.CAMERA_WIDTH - line2x, Vision.CAMERA_HEIGHT));

        // Averaging the colors in the zones
        avgColor1 = Core.mean(zone1);
        avgColor2 = Core.mean(zone2);
        avgColor3 = Core.mean(zone3);

        // Putting averaged colors on zones (we can see on camera now)
        zone1.setTo(avgColor1);
        zone2.setTo(avgColor2);
        zone3.setTo(avgColor3);

        distance1 = color_distance(avgColor1, ELEMENT_COLOR);
        distance2 = color_distance(avgColor2, ELEMENT_COLOR);
        distance3 = color_distance(avgColor3, ELEMENT_COLOR);
        max_distance = Math.max(distance3, Math.max(distance1, distance2));

        return frame;
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        // no-op
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
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        // no-op
    }

    @Override
    public String getName() {
        return "teamprop";
    }

    @Override
    public void update() {
        data.clear();
        if (max_distance == distance1) {
            data.add(new TeamPropData(Positions.RIGHT, distance1, distance2, distance3, max_distance));
        } else if ((distance1 + distance3) / distance2 > 9) {
            // The ratio of distances will be significantly different if the element is in the center
            data.add(new TeamPropData(Positions.CENTER, distance1, distance2, distance3, max_distance));
        } else {
            data.add(new TeamPropData(Positions.LEFT, distance1, distance2, distance3, max_distance));
        }
    }

    public enum Positions {
        LEFT,
        CENTER,
        RIGHT
    }
}