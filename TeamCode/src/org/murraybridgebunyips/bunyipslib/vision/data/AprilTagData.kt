package org.murraybridgebunyips.bunyipslib.vision.data

import org.firstinspires.ftc.robotcore.external.matrices.VectorF
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseRaw
import org.opencv.core.Point

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
    @Suppress("ArrayInDataClass")
    val corners: Array<Point>?,
    /**
     * Text name for this tag, if available.
     */
    val label: String?,
    /**
     * The physical size of the tag in the real world (measured black edge to black edge), if available.
     */
    val tagsize: Double?,
    /**
     * A vector describing the tag's 3D translation on the field, if available.
     */
    val fieldPosition: VectorF?,
    /**
     * A quaternion describing the tag's orientation on the field, if available.
     */
    val fieldOrientation: Quaternion?,
    /**
     * The distance unit used by the fieldPosition vector and tagsize, if available.
     */
    val distanceUnit: DistanceUnit?,
    /**
     * X translation of AprilTag, relative to camera lens. Measured sideways (Horizontally in camera image)
     * the positive X axis extends out to the right of the camera viewpoint. An x value of zero implies
     * that the Tag is centered between the left and right sides of the Camera image.
     */
    val x: Double?,
    /**
     * Y translation of AprilTag, relative to camera lens. Measured forwards (Horizontally in camera
     * image) the positive Y axis extends out in the direction the camera is pointing. A y value of
     * zero implies that the Tag is touching (aligned with) the lens of the camera, which is physically
     * unlikely. This value should always be positive.
     */
    val y: Double?,
    /**
     * Z translation of AprilTag, relative to camera lens. Measured upwards (Vertically in camera image)
     * the positive Z axis extends Upwards in the camera viewpoint. A z value of zero implies that
     * the Tag is centered between the top and bottom of the camera image.
     */
    val z: Double?,
    /**
     * Rotation of AprilTag around the X axis. Right-Hand-Rule defines positive Pitch rotation as
     * the Tag Image face twisting down when viewed from the camera. A pitch value of zero implies
     * that the camera is directly in front of the Tag, as viewed from the side.
     */
    val pitch: Double?,
    /**
     * Rotation of AprilTag around the Y axis. Right-Hand-Rule defines positive Roll rotation as the
     * Tag Image rotating Clockwise when viewed from the camera. A roll value of zero implies that
     * the Tag image is aligned squarely and upright, when viewed in the camera image frame.
     */
    val roll: Double?,
    /**
     * Rotation of AprilTag around the Z axis. Right-Hand-Rule defines positive Yaw rotation as
     * Counter-Clockwise when viewed from above. A yaw value of zero implies that the camera is
     * directly in front of the Tag, as viewed from above.
     */
    val yaw: Double?,
    /**
     * Range, (Distance), from the Camera lens to the center of the Tag, as
     * measured along the X-Y plane (across the ground).
     */
    val range: Double?,
    /**
     * Bearing, or Horizontal Angle, from the "camera center-line", to the "line joining the Camera
     * lens and the Center of the Tag". This angle is measured across the X-Y plane (across the ground).
     * A positive Bearing indicates that the robot must employ a positive Yaw (rotate counter clockwise)
     * in order to point towards the target.
     */
    val bearing: Double?,
    /**
     * Elevation, (Vertical Angle), from "the camera center-line", to "the line joining the Camera
     * Lens and the Center of the Tag". A positive Elevation indicates that the robot must employ
     * a positive Pitch (tilt up) in order to point towards the target.
     */
    val elevation: Double?,
    /**
     * Raw translation vector and orientation matrix returned by the pose solver.
     * This is not useful for most applications, but can be used to compute custom projections.
     */
    val rawPose: AprilTagPoseRaw?,
    /**
     * Timestamp of when the image in which this detection was found was acquired.
     */
    val frameAcquisitionNanoTime: Long
) : VisionData()
