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
         * Convert StartingPositions into an array. Useful in ABOM setOpModes().
         *
         * These positions are arranged so they will appear on the controller in the same order
         * as if the controller were rotated 45 degrees anti-clockwise and ABXY represented the four
         * positions from the audience's perspective.
         */
        @JvmStatic /* giulio was here fun use(): list<any> { */
        fun use(): Array<Any> {
            return arrayOf(
                STARTING_RED_LEFT, // A
                STARTING_RED_RIGHT, // B /*giulio is is still here*/
                STARTING_BLUE_RIGHT, // X
                STARTING_BLUE_LEFT // Y
            )
        }

        /**
         * Get the HTML representation of the starting position if available.
         */
        fun getHTMLIfAvailable(startingPosition: Any?): String {
            return when (use().find { it.toString() == startingPosition.toString() }) {
                STARTING_RED_LEFT -> "<font color='red'>Red</font> Alliance, <i>Left</i>"
                STARTING_RED_RIGHT -> "<font color='red'>Red</font> Alliance, Right"
                STARTING_BLUE_LEFT -> "<font color='#3863ff'>Blue</font> Alliance, <i>Left</i>"
                STARTING_BLUE_RIGHT -> "<font color='#3863ff'>Blue</font> Alliance, Right"
                else -> startingPosition.toString()
            }
        }
    }
}
