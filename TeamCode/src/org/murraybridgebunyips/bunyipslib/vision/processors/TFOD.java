package org.murraybridgebunyips.bunyipslib.vision.processors;

import android.graphics.Canvas;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import org.murraybridgebunyips.bunyipslib.vision.Processor;
import org.murraybridgebunyips.bunyipslib.vision.Vision;
import org.murraybridgebunyips.bunyipslib.vision.data.TfodData;
import org.opencv.core.Mat;

import java.util.List;

/**
 * Extension wrapper for TFOD to interop with the Vision system
 *
 * @author Lucas Bubner, 2023
 */
public class TFOD extends Processor<TfodData> {
    private final TfodProcessor instance;
    private volatile Object tfodCtx;

    /**
     * Create a new TFOD processor
     */
    public TFOD() {
        instance = new TfodProcessor.Builder()
                // Specify custom TFOD settings here
                // By default this will load the current season assets
                .build();
    }

    /**
     * Get the TFOD instance
     *
     * @return direct TFODProcessor instance
     */
    public TfodProcessor getInstance() {
        return instance;
    }

    @NonNull
    @Override
    public String toString() {
        return "tfod";
    }

    @Override
    public void update() {
        List<Recognition> recognitions = instance.getRecognitions();
        for (Recognition recognition : recognitions) {
            data.add(new TfodData(
                    recognition.getLabel(),
                    recognition.getConfidence(),
                    recognition.getLeft(),
                    recognition.getTop(),
                    recognition.getRight(),
                    recognition.getBottom(),
                    recognition.getWidth(),
                    recognition.getHeight(),
                    recognition.getImageWidth(),
                    recognition.getImageHeight(),
                    recognition.estimateAngleToObject(AngleUnit.RADIANS)
            ));
        }
    }

    // Untouched methods to leave to the TFODProcessor instance, TFOD
    // processing is already handled by the SDK

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        instance.init(width, height, calibration);
    }

    @Override
    public void onProcessFrame(Mat frame, long captureTimeNanos) {
        tfodCtx = instance.processFrame(frame, captureTimeNanos);
    }

    @Override
    public void onFrameDraw(Canvas canvas) {
        instance.onDrawFrame(canvas, Vision.CAMERA_WIDTH, Vision.CAMERA_HEIGHT, 1.0f, 1.0f, tfodCtx);
    }
}
