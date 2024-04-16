package org.murraybridgebunyips.bunyipslib.vision;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.murraybridgebunyips.bunyipslib.BunyipsOpMode;
import org.murraybridgebunyips.bunyipslib.vision.data.ContourData;
import org.murraybridgebunyips.bunyipslib.vision.data.VisionData;
import org.murraybridgebunyips.bunyipslib.vision.processors.ColourThreshold;
import org.opencv.core.Mat;
import org.opencv.core.Rect;

/**
 * An automatic tuning OpMode for calibrating the vision system's colour thresholding for one processor.
 *
 * @author Lucas Bubner, 2024
 */
public abstract class AutoContourTuner extends BunyipsOpMode {
    private RectangleAid aid;
    private ColourThreshold processor;
    private Vision vision;
    private Rect fittingRect = new Rect(
            Vision.CAMERA_WIDTH / 3,
            Vision.CAMERA_HEIGHT / 3,
            // Default of 1/3 of the camera view
            Vision.CAMERA_WIDTH / 3,
            Vision.CAMERA_HEIGHT / 3
    );

    protected abstract ColourThreshold setThresholdToTune();

    protected abstract CameraName setCamera();

    @Nullable
    protected abstract Rect setCustomFittingRect();

    @Override
    protected void onInit() {
        processor = setThresholdToTune();

        Rect customRect = setCustomFittingRect();
        if (customRect != null)
            fittingRect = customRect;

        vision = new Vision(setCamera());
        aid = new RectangleAid();

        vision.init(processor, aid);
        vision.start(processor, aid);
        vision.startPreview();
        vision.setPreview(aid);

        telemetry.add("Please place the object to tune the threshold for in the camera view, in the box highlighted on the preview.");
    }

    @Override
    protected void onStart() {
        vision.stop(aid);
        vision.setPreview(processor);
    }

    @Override
    protected void activeLoop() {
        vision.update();

        ContourData biggestContour = ContourData.getLargest(processor.getData());
        if (biggestContour == null) return;

        // TODO: fitting bounding boxes positions and size
    }

    private class RectangleAid extends Processor<VisionData> {
        private final ElapsedTime timer = new ElapsedTime();

        @NonNull
        @Override
        public String toString() {
            return "rectangleaid";
        }

        @Override
        public void init(int width, int height, CameraCalibration calibration) {
            timer.reset();
        }

        @Override
        protected void update() {
            // no-op
        }

        @Override
        protected void onProcessFrame(Mat frame, long captureTimeNanos) {
            // no-op
        }

        @Override
        protected void onFrameDraw(Canvas canvas) {
            // Flash once per second
            if (timer.seconds() <= 1) {
                canvas.drawRect(
                        fittingRect.x,
                        fittingRect.y,
                        fittingRect.x + fittingRect.width,
                        fittingRect.y + fittingRect.height,
                        new Paint() {{
                            setColor(Color.RED);
                            setStyle(Style.STROKE);
                            setStrokeWidth(5);
                        }}
                );
            } else if (timer.seconds() >= 2) {
                timer.reset();
            }
        }
    }
}
