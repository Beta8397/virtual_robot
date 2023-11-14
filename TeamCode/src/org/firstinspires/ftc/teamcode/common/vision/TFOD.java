package org.firstinspires.ftc.teamcode.common.vision;

import android.graphics.Canvas;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.common.vision.data.TfodData;
import org.opencv.core.Mat;

/**
 * Extension wrapper for TFOD to interop with the Vision system
 *
 * @author Lucas Bubner, 2023
 */
public class TFOD extends Processor<TfodData> {

    /**
     * Vision Processor Wrapper
     * Parameterized type T must be a subclass extension of VisionData and getName must return a non-null value
     * Super-call: {@code super([yourVisionDataClass].class)}
     */
    protected TFOD() {
        super(TfodData.class);
        // This class is noop as Vision is impossible
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }

    @Override
    public String getName() {
        return null;
    }

    @Override
    public void tick() {

    }
}
