package org.murraybridgebunyips.bunyipslib

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import org.murraybridgebunyips.bunyipslib.external.units.Angle
import org.murraybridgebunyips.bunyipslib.external.units.Measure
import org.murraybridgebunyips.bunyipslib.external.units.Units.Radians

/**
 * Represents relative positions in 2D space.
 * The origin of these vectors face the X axis, for continuity.
 *
 * +X is forward, +Y is left, +heading is anticlockwise.
 * Migrated to use Pose2d from RoadRunner 13/11/2023.
 * @author Lucas Bubner, 2023
 */
@Suppress("KDocMissingDocumentation")
enum class Direction(
    /**
     * The vector of the relative position.
     */
    val vector: Vector2d
) {
    ZERO(Vector2d(0.0, 0.0)),

    // 2D translational vectors
    FORWARD(Vector2d(1.0, 0.0)),
    BACKWARD(Vector2d(-1.0, 0.0)),
    LEFT(Vector2d(0.0, 1.0)),
    RIGHT(Vector2d(0.0, -1.0)),

    // Fancy 2D translational vectors
    FORWARD_LEFT(Vector2d(1.0, 1.0)),
    FORWARD_RIGHT(Vector2d(1.0, -1.0)),
    BACKWARD_LEFT(Vector2d(-1.0, 1.0)),
    BACKWARD_RIGHT(Vector2d(-1.0, -1.0));

    // 2D rotational vectors
    enum class Rotation(val pose: Pose2d) {
        ZERO(Pose2d(0.0, 0.0, 0.0)),
        CLOCKWISE(Pose2d(0.0, 0.0, Math.toRadians(-90.0))),
        ANTICLOCKWISE(Pose2d(0.0, 0.0, Math.toRadians(90.0)));
    }

    val angle: Measure<Angle>
        get() = Radians.of(vector.angle())

    companion object {
        /**
         * Convert a Pose2d robot vector to a relative vector.
         */
        @JvmStatic
        fun convert(pose: Pose2d): Any {
            if (pose.heading != 0.0) {
                return when (pose.heading) {
                    Rotation.CLOCKWISE.pose.heading -> Rotation.CLOCKWISE
                    Rotation.ANTICLOCKWISE.pose.heading -> Rotation.ANTICLOCKWISE
                    else -> throw IllegalArgumentException("RelativeVector2d: (${pose.x},${pose.y},${pose.heading}) cannot be converted to a Vector2d.")
                }
            }
            return convert(Vector2d(pose.x, pose.y))
        }

        /**
         * Convert a Vector2d robot vector into a relative vector.
         */
        @JvmStatic
        fun convert(vector: Vector2d): Direction {
            return when (vector) {
                FORWARD.vector -> FORWARD
                BACKWARD.vector -> BACKWARD
                LEFT.vector -> LEFT
                RIGHT.vector -> RIGHT
                FORWARD_LEFT.vector -> FORWARD_LEFT
                FORWARD_RIGHT.vector -> FORWARD_RIGHT
                BACKWARD_LEFT.vector -> BACKWARD_LEFT
                BACKWARD_RIGHT.vector -> BACKWARD_RIGHT
                else -> throw IllegalArgumentException("RelativeVector2d: (${vector.x},${vector.y}) cannot be converted to a Vector2d.")
            }
        }
    }
}