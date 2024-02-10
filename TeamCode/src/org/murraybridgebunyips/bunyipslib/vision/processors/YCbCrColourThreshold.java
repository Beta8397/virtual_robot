package org.murraybridgebunyips.bunyipslib.vision.processors;

import android.graphics.Canvas;
import android.graphics.Paint;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.murraybridgebunyips.bunyipslib.vision.Processor;
import org.murraybridgebunyips.bunyipslib.vision.Vision;
import org.murraybridgebunyips.bunyipslib.vision.data.ContourData;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

/**
 * Colour thresholding processor for YCbCr colour space, used to find colour contours in an image.
 *
 * @author Lucas Bubner, 2024
 */
@Config
public abstract class YCbCrColourThreshold extends Processor<ContourData> {
    public static double CONTOUR_AREA_THRESHOLD_PERCENT = 20;

    private final Scalar lower = getLower();
    private final Scalar upper = getUpper();
    private final Mat ycrcbMat = new Mat();
    private final Mat binaryMat = new Mat();
    private final Mat maskedInputMat = new Mat();
    private final List<MatOfPoint> contours = new ArrayList<>();
    private final Mat hierarchy = new Mat();

    public abstract Scalar getLower();

    public abstract Scalar getUpper();

    @Override
    public void update() {
        for (MatOfPoint contour : contours) {
            Rect boundingRect = Imgproc.boundingRect(contour);
            if (boundingRect.area() < (CONTOUR_AREA_THRESHOLD_PERCENT / 100) * (Vision.CAMERA_WIDTH * Vision.CAMERA_HEIGHT))
                continue;
            data.add(new ContourData(boundingRect));
        }
    }

    @Override
    public final Object onProcessFrame(Mat frame, long captureTimeNanos) {
        /*
         * Converts our input mat from RGB to
         * specified color space by the enum.
         * EOCV ALWAYS returns RGB mats, so you'd
         * always convert from RGB to the color
         * space you want to use.
         *
         * Takes our "input" mat as an input, and outputs
         * to a separate Mat buffer "ycrcbMat"
         */
        Imgproc.cvtColor(frame, ycrcbMat, Imgproc.COLOR_RGB2YCrCb);

        /*
         * This is where our thresholding actually happens.
         * Takes our "ycrcbMat" as input and outputs a "binary"
         * Mat to "binaryMat" of the same size as our input.
         * "Discards" all the pixels outside the bounds specified
         * by the scalars "lower" and "upper".
         *
         * Binary meaning that we have either a 0 or 255 value
         * for every pixel.
         *
         * 0 represents our pixels that were outside the bounds
         * 255 represents our pixels that are inside the bounds
         */
        Core.inRange(ycrcbMat, lower, upper, binaryMat);

        /*
         * Release the reusable Mat so that old data doesn't
         * affect the next step in the current processing
         */
        maskedInputMat.release();

        /*
         * Now, with our binary Mat, we perform a "bitwise and"
         * to our input image, meaning that we will perform a mask
         * which will include the pixels from our input Mat which
         * are "255" in our binary Mat (meaning that they're inside
         * the range) and will discard any other pixel outside the
         * range (RGB 0, 0, 0. All discarded pixels will be black)
         */
        Core.bitwise_and(frame, frame, maskedInputMat, binaryMat);

        /*
         * Find the contours of the binary Mat. This will
         * populate the "contours" list with the contours
         * found in the binary Mat.
         */
        contours.clear();
        Imgproc.findContours(binaryMat, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        /*
         * Copy the masked input Mat to the frame
         * so that we can see the masked input on the preview
         */
        maskedInputMat.copyTo(frame);
        return frame;
    }

    // These methods are optionally overridden by the user
    // as they are often not used or needed

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
    }

    @Override
    public void onFrameDraw(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        // Draw borders around the contours
        synchronized (data) {
            for (ContourData contour : data) {
                canvas.drawRect(
                        contour.getBoundingRect().x * scaleBmpPxToCanvasPx,
                        contour.getBoundingRect().y * scaleBmpPxToCanvasPx,
                        (contour.getBoundingRect().x + contour.getBoundingRect().width) * scaleBmpPxToCanvasPx,
                        (contour.getBoundingRect().y + contour.getBoundingRect().height) * scaleBmpPxToCanvasPx,
                        new Paint() {{
                            setColor(0xFF00FF00);
                            setStyle(Style.STROKE);
                            setStrokeWidth(5);
                        }}
                );
            }
        }
    }
}
