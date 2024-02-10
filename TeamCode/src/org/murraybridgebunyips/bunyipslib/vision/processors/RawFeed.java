package org.murraybridgebunyips.bunyipslib.vision.processors;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.murraybridgebunyips.bunyipslib.vision.Processor;
import org.murraybridgebunyips.bunyipslib.vision.data.VisionData;
import org.opencv.core.Mat;

public class RawFeed extends Processor<VisionData> {
    @Override
    public String getName() {
        return "rawfeed";
    }

    @Override
    public void update() {
        // no-op
    }

    @Override
    public Object onProcessFrame(Mat frame, long captureTimeNanos) {
        return frame;
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        // no-op
    }

    @Override
    public void onFrameDraw(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        // no-op
    }
}
