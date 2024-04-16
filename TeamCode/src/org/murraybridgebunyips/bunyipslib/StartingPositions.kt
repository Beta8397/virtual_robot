package org.murraybridgebunyips.bunyipslib

import com.acmerobotics.roadrunner.geometry.Vector2d

/**
 * Enum for determining where the robot is starting on the field. This can be used to determine
 * which autonomous path to take.
 */
enum class StartingPositions(
    /**
     * The vector of the starting position.
     */
    val vector: Vector2d
) {
    /**
     * FTC Field starting position Red Alliance, Left side if viewed from the Red Alliance.
     */
    STARTING_RED_LEFT(Vector2d(-36.0, -60.0)),

    /**
     * FTC Field starting position Red Alliance, Right side if viewed from the Red Alliance.
     */
    STARTING_RED_RIGHT(Vector2d(12.0, -60.0)),

    /**
     * FTC Field starting position Blue Alliance, Left side if viewed from the Blue Alliance.
     */
    STARTING_BLUE_LEFT(Vector2d(12.0, 60.0)),

    /**
     * FTC Field starting position Blue Alliance, Right side if viewed from the Blue Alliance.
     */
    STARTING_BLUE_RIGHT(Vector2d(-36.0, 60.0));

    companion object {
        /**
         * Convert StartingPositions into a list. Useful in ABOM setOpModes().
         *
         * These positions are arranged so they will appear on the controller in the same order
         * as if the controller were rotated 45 degrees anti-clockwise and ABXY represented the four
         * positions from the audience's perspective.
         */
        @JvmStatic
        fun use(): List<StartingPositions> {
            return listOf(
                STARTING_RED_LEFT, // A
                STARTING_RED_RIGHT, // B
                STARTING_BLUE_RIGHT, // X
                STARTING_BLUE_LEFT // Y
            )
        }
    }
}
