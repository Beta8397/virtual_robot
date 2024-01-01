package org.murraybridgebunyips.bunyipslib

import com.acmerobotics.roadrunner.geometry.Pose2d

/**
 * Represents relative Pose2d in 2D space.
 * The origin of these vectors face the X axis, for continuity.
 *
 * +X is forward, +Y is left, +heading is anticlockwise.
 * Migrated to use Pose2d from RoadRunner 13/11/2023.
 * @author Lucas Bubner, 2023
 */
enum class RelativePose2d(val vector: Pose2d) {
    // 2D translational vectors
    FORWARD(Pose2d(1.0, 0.0, 0.0)),
    BACKWARD(Pose2d(-1.0, 0.0, 0.0)),
    LEFT(Pose2d(0.0, 1.0, 0.0)),
    RIGHT(Pose2d(0.0, -1.0, 0.0)),

    // Fancy 2D translational vectors
    FORWARD_LEFT(Pose2d(1.0, 1.0, 0.0)),
    FORWARD_RIGHT(Pose2d(1.0, -1.0, 0.0)),
    BACKWARD_LEFT(Pose2d(-1.0, 1.0, 0.0)),
    BACKWARD_RIGHT(Pose2d(-1.0, -1.0, 0.0)),

    // 2D rotational vectors
    CLOCKWISE(Pose2d(0.0, 0.0, Math.toRadians(-90.0))),
    ANTICLOCKWISE(Pose2d(0.0, 0.0, Math.toRadians(90.0)));

    val degrees: Double
        get() = Math.toDegrees(vector.headingVec().angle())

    val radians: Double
        get() = vector.headingVec().angle()

    companion object {
        /**
         * Convert a Pose2d robot vector to a relative vector.
         */
        @JvmStatic
        fun convert(vector: Pose2d): RelativePose2d {
            return when (vector) {
                FORWARD.vector -> FORWARD
                BACKWARD.vector -> BACKWARD
                LEFT.vector -> LEFT
                RIGHT.vector -> RIGHT
                FORWARD_LEFT.vector -> FORWARD_LEFT
                FORWARD_RIGHT.vector -> FORWARD_RIGHT
                BACKWARD_LEFT.vector -> BACKWARD_LEFT
                BACKWARD_RIGHT.vector -> BACKWARD_RIGHT
                CLOCKWISE.vector -> CLOCKWISE
                ANTICLOCKWISE.vector -> ANTICLOCKWISE
                else -> throw IllegalArgumentException("RelativePose2d: (${vector.x},${vector.y},${vector.heading}) cannot be converted to a Pose2d.")
            }
        }
    }
}