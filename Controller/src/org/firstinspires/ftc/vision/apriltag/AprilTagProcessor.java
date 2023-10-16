package org.firstinspires.ftc.vision.apriltag;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Mat;

import java.util.Collections;
import java.util.List;

public class AprilTagProcessor implements VisionProcessor {
    public static AprilTagProcessor easyCreateWithDefaults(){
        return new AprilTagProcessor();
    }
    public List<AprilTagDetection> getDetections(){
        return null;
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
}
