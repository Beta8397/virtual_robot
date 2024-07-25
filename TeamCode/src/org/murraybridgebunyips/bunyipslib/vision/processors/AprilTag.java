package org.murraybridgebunyips.bunyipslib.vision.processors;

import android.graphics.Canvas;
import android.util.Size;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.murraybridgebunyips.bunyipslib.cameras.CameraType;
import org.murraybridgebunyips.bunyipslib.vision.Processor;
import org.murraybridgebunyips.bunyipslib.vision.data.AprilTagData;
import org.opencv.core.Mat;

import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import java.util.function.Function;

/**
 * AprilTag Detection Processor
 * <p>
 * This is an extension wrapper for the SDK-provided AprilTagProcessor to interop with the Vision system.
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

    /**
     * Create an AprilTag instance with the additional builder settings as provided.
     * Will rely on the SDK to provide camera intrinsics for this overload.
     *
     * @param customBuilderSettings additional settings that will be attached to the builder
     */
    public AprilTag(Function<AprilTagProcessor.Builder, AprilTagProcessor.Builder> customBuilderSettings) {
        instance = customBuilderSettings.apply(makeBuilderWithCommonSettings()).build();
    }

    /**
     * Create an AprilTag instance with the additional builder settings as provided.
     * Will use the provided camera calibration.
     *
     * @param customBuilderSettings additional settings that will be attached to the builder
     * @param camInfo               CameraType instance with the camera calibration
     */
    public AprilTag(Function<AprilTagProcessor.Builder, AprilTagProcessor.Builder> customBuilderSettings, CameraType camInfo) {
        instance = customBuilderSettings.apply(
                makeBuilderWithCommonSettings()
                        .setLensIntrinsics(camInfo.getFx(), camInfo.getFy(), camInfo.getCx(), camInfo.getCy())
        ).build();
    }

    private AprilTagProcessor.Builder makeBuilderWithCommonSettings() {
        return new AprilTagProcessor.Builder()
                // Common (always enabled) settings
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

    @NonNull
    @Override
    public String toString() {
        return "apriltag";
    }

    @Override
    protected void update() {
        List<AprilTagDetection> detections = instance.getDetections();
        for (AprilTagDetection detection : detections) {
            // Need to wrap the AprilTagDetection in an extension of VisionData for consistency
            data.add(new AprilTagData(
                    detection.id,
                    detection.hamming,
                    detection.decisionMargin,
                    detection.center,
                    Arrays.asList(detection.corners),
                    Optional.ofNullable(detection.metadata),
                    Optional.ofNullable(detection.ftcPose),
                    Optional.ofNullable(detection.rawPose),
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
    protected void onProcessFrame(Mat frame, long captureTimeNanos) {
        atCtx = instance.processFrame(frame, captureTimeNanos);
    }

    @Override
    protected void onFrameDraw(Canvas canvas) {
        Size dimensions = getCameraDimensions();
        if (dimensions == null) return;
        instance.onDrawFrame(canvas, dimensions.getWidth(), dimensions.getHeight(), 1.0f, 1.0f, atCtx);
    }
}
