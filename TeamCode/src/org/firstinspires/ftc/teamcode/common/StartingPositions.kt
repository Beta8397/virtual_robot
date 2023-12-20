package org.firstinspires.ftc.teamcode.common

/**
 * Enum for determining where the robot is starting on the field. This can be used to determine
 * which autonomous path to take.
 */
enum class StartingPositions {
    STARTING_RED_LEFT, STARTING_RED_RIGHT, STARTING_BLUE_LEFT, STARTING_BLUE_RIGHT;

    companion object {
        /**
         * Convert StartingPositions into a list of OpModeSelections. Useful in ABOM setOpModes().
         */
        @JvmStatic
        fun use(): List<OpModeSelection> {
            return listOf(
                OpModeSelection(STARTING_RED_LEFT),
                OpModeSelection(STARTING_RED_RIGHT),
                OpModeSelection(STARTING_BLUE_LEFT),
                OpModeSelection(STARTING_BLUE_RIGHT)
            )
        }
    }
}
