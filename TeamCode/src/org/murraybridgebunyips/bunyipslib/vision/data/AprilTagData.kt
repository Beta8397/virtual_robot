package org.murraybridgebunyips.bunyipslib.vision.data

import org.firstinspires.ftc.vision.apriltag.AprilTagMetadata
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseRaw
import org.opencv.core.Point
import java.util.Optional

/**
 * Utility data structure for AprilTag Detections.
 */
data class AprilTagData(
    /**
     * AprilTag ID.
     */
    val id: Int,
    /**
     * The number of bits corrected when reading the tag ID payload.
     */
    val hamming: Int,
    /**
     * How much margin remains before the detector would decide to reject a tag.
     */
    val decisionMargin: Float,
    /**
     * The image pixel coordinates of the center of the tag.
     */
    val center: Point,
    /**
     * The image pixel coordinates of the corners of the tag.
     */
    val corners: List<Point>,
    /**
     * Metadata pertaining to this AprilTag as specified by the FTC SDK.
     * Optional value may be `null` depending on the used AprilTag Library.
     *
     * *Properties:*
     *
     * `id` the ID of the tag
     *
     * `name` a text name for the tag
     *
     * `tagsize` the physical size of the tag in the real world (measured black edge to black edge)
     *
     * `fieldPosition` a vector describing the tag's 3d translation on the field
     *
     * `distanceUnit` the units used for size and fieldPosition
     *
     * `fieldOrientation` a quaternion describing the tag's orientation on the field
     */
    val metadata: Optional<AprilTagMetadata>,
    /**
     * 6DOF pose data formatted in useful ways for FTC gameplay.
     * Optional value may be `null` depending on the used AprilTag Library.
     * Units from this pose will be in *inches* and *degrees*.
     */
    val ftcPose: Optional<AprilTagPoseFtc>,
    /**
     * Raw translation vector and orientation matrix returned by the pose solver.
     * Optional value may be `null` depending on the used AprilTag Library.
     * This is the backing pose that [ftcPose] uses internally.
     */
    val rawPose: Optional<AprilTagPoseRaw>,
    /**
     * Timestamp of when the image in which this detection was found was acquired.
     */
    val frameAcquisitionNanoTime: Long
) : VisionData() {
    /**
     * Check if this AprilTag has metadata and pose information available, as it is present in the currently used library.
     */
    fun isInLibrary(): Boolean {
        return metadata.isPresent && ftcPose.isPresent && rawPose.isPresent
    }
}
