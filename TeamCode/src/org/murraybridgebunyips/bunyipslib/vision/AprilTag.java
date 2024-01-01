package org.murraybridgebunyips.bunyipslib.vision;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.murraybridgebunyips.bunyipslib.cameras.CameraType;
import org.murraybridgebunyips.bunyipslib.vision.data.AprilTagData;
import org.opencv.core.Mat;

import java.util.List;

/**
 * Extension wrapper for AprilTag to interop with the Vision system
 *
 * @author Lucas Bubner, 2023
 */
public class AprilTag extends Processor<AprilTagData> {
    private final AprilTagProcessor instance;

    public AprilTag(CameraType camInfo) {
        super(AprilTagData.class);
        instance = new AprilTagProcessor.Builder()
                .setLensIntrinsics(camInfo.getFx(), camInfo.getFy(), camInfo.getCx(), camInfo.getCy())
                // Specify custom AprilTag settings here
                // By default this will load the current season assets
                .build();
    }

    /**
     * Get the AprilTag instance
     *
     * @return direct AprilTagProcessor instance
     */
    public AprilTagProcessor getInstance() {
        return instance;
    }

    @Override
    public String getName() {
        return "apriltag";
    }

    @Override
    public void tick() {
        List<AprilTagDetection> detections = instance.getFreshDetections();
        if (detections == null) {
            return;
        }
        data.clear();
        for (AprilTagDetection detection : detections) {
            data.add(new AprilTagData(
                    detection.id,
                    detection.hamming,
                    detection.decisionMargin,
                    detection.center,
                    detection.corners,
                    detection.metadata != null ? detection.metadata.name : null,
                    detection.metadata != null ? detection.metadata.tagsize : null,
                    detection.metadata != null ? detection.metadata.fieldPosition : null,
                    detection.metadata != null ? detection.metadata.fieldOrientation : null,
                    detection.metadata != null ? detection.metadata.distanceUnit : null,
                    detection.ftcPose != null ? detection.ftcPose.x : null,
                    detection.ftcPose != null ? detection.ftcPose.y : null,
                    detection.ftcPose != null ? detection.ftcPose.z : null,
                    detection.ftcPose != null ? detection.ftcPose.pitch : null,
                    detection.ftcPose != null ? detection.ftcPose.roll : null,
                    detection.ftcPose != null ? detection.ftcPose.yaw : null,
                    detection.ftcPose != null ? detection.ftcPose.range : null,
                    detection.ftcPose != null ? detection.ftcPose.bearing : null,
                    detection.ftcPose != null ? detection.ftcPose.elevation : null,
                    detection.rawPose,
                    detection.frameAcquisitionNanoTime
            ));
        }
    }

    // Untouched methods to be handled by the AprilTagProcessor

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        instance.init(width, height, calibration);
    }

    @Override
    public Object onProcessFrame(Mat frame, long captureTimeNanos) {
        return instance.processFrame(frame, captureTimeNanos);
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        instance.onDrawFrame(canvas, onscreenWidth, onscreenHeight, scaleBmpPxToCanvasPx, scaleCanvasDensity, userContext);
    }
}
