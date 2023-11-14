package org.firstinspires.ftc.teamcode.common

import com.acmerobotics.roadrunner.geometry.Pose2d

/**
 * Represents relative Pose2d in 2D space.
 * The origin of these vectors face the X axis, for continuity.
 *
 * +X is forward, +Y is left, +heading is clockwise.
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
    CLOCKWISE(Pose2d(0.0, 0.0, Math.toRadians(90.0))),
    ANTICLOCKWISE(Pose2d(0.0, 0.0, Math.toRadians(-90.0)));

    val degrees: Double
        get() = Math.toDegrees(vector.headingVec().angle())

    val radians: Double
        get() = vector.headingVec().angle()

    companion object {
        /**
         * Convert a robot vector to a relative vector.
         */
        fun convert(vector: Pose2d): RelativePose2d {
            return when {
                vector.x > 0.5 && vector.y > 0.5 -> FORWARD_RIGHT
                vector.x > 0.5 && vector.y < -0.5 -> BACKWARD_RIGHT
                vector.x < -0.5 && vector.y > 0.5 -> FORWARD_LEFT
                vector.x < -0.5 && vector.y < -0.5 -> BACKWARD_LEFT
                vector.x > 0.5 -> RIGHT
                vector.x < -0.5 -> LEFT
                vector.y > 0.5 -> FORWARD
                vector.y < -0.5 -> BACKWARD
                vector.heading > 0 -> CLOCKWISE
                vector.heading < 0 -> ANTICLOCKWISE
                else -> throw IllegalArgumentException("RelativePose2d: (${vector.x},${vector.y},${vector.heading}) cannot be converted to a Pose2d.")
            }
        }
    }
}