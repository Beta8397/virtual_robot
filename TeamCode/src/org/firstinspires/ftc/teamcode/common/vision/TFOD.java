package org.firstinspires.ftc.teamcode.common.vision;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
//import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import org.firstinspires.ftc.teamcode.common.vision.data.TfodData;
import org.opencv.core.Mat;

import java.util.List;

/**
 * Extension wrapper for TFOD to interop with the Vision system
 *
 * @author Lucas Bubner, 2023
 */
public class TFOD extends Processor<TfodData> {
//    private final TfodProcessor instance;

    public TFOD() {
        super(TfodData.class);
//        instance = new TfodProcessor.Builder()
//                // Specify custom TFOD settings here
//                // By default this will load the current season assets
//                .build();
    }

    /**
     * Get the TFOD instance
     *
     * @return direct TFODProcessor instance
     */
//    public TfodProcessor getInstance() {
//        return instance;
//    }

    @Override
    public String getName() {
        return "tfod";
    }

    public void tick() {
//        List<Recognition> recognitions = instance.getFreshRecognitions();
//        if (recognitions == null) {
//            return;
//        }
        data.clear();
//        for (Recognition recognition : recognitions) {
//            data.add(new TfodData(
//                    recognition.getLabel(),
//                    recognition.getConfidence(),
//                    recognition.getLeft(),
//                    recognition.getTop(),
//                    recognition.getRight(),
//                    recognition.getBottom(),
//                    recognition.getWidth(),
//                    recognition.getHeight(),
//                    recognition.getImageWidth(),
//                    recognition.getImageHeight(),
//                    recognition.estimateAngleToObject(AngleUnit.DEGREES),
//                    recognition.estimateAngleToObject(AngleUnit.RADIANS)
//            ));
//        }
    }

    // Untouched methods to leave to the TFODProcessor instance, TFOD
    // processing is already handled by the SDK

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
//        instance.init(width, height, calibration);
    }

    @Override
    public Object onProcessFrame(Mat frame, long captureTimeNanos) {
//        return instance.processFrame(frame, captureTimeNanos);
        return frame;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
//        instance.onDrawFrame(canvas, onscreenWidth, onscreenHeight, scaleBmpPxToCanvasPx, scaleCanvasDensity, userContext);
    }
}
