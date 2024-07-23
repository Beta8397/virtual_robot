package org.murraybridgebunyips.bunyipslib.vision;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.vision.apriltag.AprilTagMetadata;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.murraybridgebunyips.bunyipslib.Dbg;
import org.murraybridgebunyips.bunyipslib.roadrunner.drive.RoadRunnerDrive;
import org.murraybridgebunyips.bunyipslib.vision.data.AprilTagData;
import org.murraybridgebunyips.bunyipslib.vision.processors.AprilTag;

import java.util.ArrayList;
import java.util.Objects;

/**
 * Combines an AprilTag processor and RoadRunner drive to supply updates in pose estimation.
 *
 * @author Lucas Bubner, 2024
 */
public class AprilTagPoseEstimator {
    private final AprilTag processor;
    private final RoadRunnerDrive drive;
    private boolean active = true;
    private boolean updateHeading;

    /**
     * Constructor for AprilTagPoseEstimator.
     * Note that the option to also update heading based off these readings is disabled by default.
     * Enable with {@link #setHeadingEstimate}.
     *
     * @param processor AprilTag processor to use for pose estimation, must be attached to Vision and running
     * @param drive     RoadRunner drive
     */
    public AprilTagPoseEstimator(AprilTag processor, RoadRunnerDrive drive) {
        this.processor = processor;
        this.drive = drive;
    }

    /**
     * Whether to set the drive pose to the vision estimate. Default is on.
     *
     * @param setPoseAutomatically whether AprilTagPoseEstimator is on
     */
    public void setActive(boolean setPoseAutomatically) {
        active = setPoseAutomatically;
    }

    /**
     * Whether to set the drive pose to the vision estimate with heading information. Default is off.
     *
     * @param setHeadingAutomatically whether to also set the heading to the AprilTag estimate or not
     */
    public void setHeadingEstimate(boolean setHeadingAutomatically) {
        updateHeading = setHeadingAutomatically;
    }

    /**
     * Propagate interpretation of AprilTag processor and set the pose estimate to the tag, if
     * available by the SDK. This method will no-op if insufficient information is available.
     */
    public void update() {
        if (!active || !processor.isRunning())
            return;

        ArrayList<AprilTagData> data = processor.getData();
        if (data.isEmpty())
            return;

        // We will simply rely on the first entry in the list of detected tags that we can use, if any
        for (int i = 0; i < data.size(); i++) {
            AprilTagData aprilTag = data.get(i);
            if (aprilTag.getMetadata() == null) {
                // No luck with this ID
                continue;
            }

            AprilTagMetadata metadata = aprilTag.getMetadata();
            AprilTagPoseFtc camPose = aprilTag.getFtcPose();

            VectorF tagPos = Objects.requireNonNull(metadata.fieldPosition);
            Orientation tagOri = Objects.requireNonNull(metadata.fieldOrientation)
                    .toOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS);

            double tagX = tagPos.get(0);
            double tagY = tagPos.get(1);
            double tagRotation = tagOri.thirdAngle;
            // x'=x*cos(t)-y*sin(t)
            // y'=x*sin(t)+y*cos(t)
            double relativeX = camPose.x * Math.cos(tagRotation) - camPose.y * Math.sin(tagRotation);
            double relativeY = camPose.x * Math.sin(tagRotation) + camPose.y * Math.cos(tagRotation);
            // Displacement vector
            Vector2d pos = new Vector2d(
                    tagX - relativeX,
                    tagY - relativeY
            );

            // Only set the heading if the user wants it, which we can do fairly simply if they want that too
            // Future: Can integrate pose info with a Kalman filter to filter out inaccurate results
            double heading = drive.getPoseEstimate().getHeading();
            if (updateHeading) {
                // Rotate 90 degrees to match unit circle proportions due to a rotation mismatch
                heading = Math.PI / 2.0 + tagRotation - Math.toRadians(camPose.yaw);
            }
            Pose2d estimatedPose = new Pose2d(pos.getX(), pos.getY(), heading);

            // Avoid spamming the logs by logging the events that are over an inch away from the current estimate
            if (drive.getPoseEstimate().vec().distTo(estimatedPose.vec()) >= 1)
                Dbg.logd(getClass(), "Updated pose based on AprilTag ID#%, %->%", aprilTag.getId(), drive.getPoseEstimate(), estimatedPose);

            drive.setPoseEstimate(estimatedPose);
            // Stop searching as we have a new pose
            break;
        }
    }
}
