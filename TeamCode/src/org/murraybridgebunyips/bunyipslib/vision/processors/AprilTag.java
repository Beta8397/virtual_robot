package org.murraybridgebunyips.bunyipslib.vision.processors;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.murraybridgebunyips.bunyipslib.cameras.CameraType;
import org.murraybridgebunyips.bunyipslib.vision.Processor;
import org.murraybridgebunyips.bunyipslib.vision.Vision;
import org.murraybridgebunyips.bunyipslib.vision.data.AprilTagData;
import org.opencv.core.Mat;

import java.util.Arrays;
import java.util.List;

/**
 * Extension wrapper for AprilTag to interop with the Vision system
 *
 * @author Lucas Bubner, 2023
 */
public class AprilTag extends Processor<AprilTagData> {
    private final AprilTagProcessor instance;
    private volatile Object atCtx;

    /**
     * Will use the provided camera calibration
     *
     * @param camInfo CameraType instance with the camera calibration
     */
    public AprilTag(CameraType camInfo) {
        instance = makeBuilderWithCommonSettings()
                .setLensIntrinsics(camInfo.getFx(), camInfo.getFy(), camInfo.getCx(), camInfo.getCy())
                .build();
    }

    /**
     * Will rely on the SDK to provide camera intrinsics
     */
    public AprilTag() {
        instance = makeBuilderWithCommonSettings().build();
    }

    private AprilTagProcessor.Builder makeBuilderWithCommonSettings() {
        return new AprilTagProcessor.Builder()
                // Specify custom AprilTag settings here, season assets and units are automatic
                .setDrawAxes(true)
                .setDrawCubeProjection(true);
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
    public void update() {
        List<AprilTagDetection> detections = instance.getDetections();
        for (AprilTagDetection detection : detections) {
            data.add(new AprilTagData(
                    detection.id,
                    detection.hamming,
                    detection.decisionMargin,
                    detection.center,
                    Arrays.asList(detection.corners),
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
    public void onProcessFrame(Mat frame, long captureTimeNanos) {
        atCtx = instance.processFrame(frame, captureTimeNanos);
    }

    @Override
    public void onFrameDraw(Canvas canvas) {
        instance.onDrawFrame(canvas, Vision.CAMERA_WIDTH, Vision.CAMERA_HEIGHT, 1.0f, 1.0f, atCtx);
    }
}
