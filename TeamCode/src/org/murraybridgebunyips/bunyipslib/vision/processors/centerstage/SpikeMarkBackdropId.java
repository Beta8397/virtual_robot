package org.murraybridgebunyips.bunyipslib.vision.processors.centerstage;

import org.murraybridgebunyips.bunyipslib.Direction;
import org.murraybridgebunyips.bunyipslib.StartingPositions;

/**
 * Util for obtaining the backdrop ID based on a Spike Mark position and field side.
 *
 * @author Lucas Bubner, 2024
 */
public final class SpikeMarkBackdropId {
    private SpikeMarkBackdropId() {
    }

    /**
     * Get the backdrop ID based on the Spike Mark position and field side.
     *
     * @param spikeMarkPosition     the position of the Spike Mark as detected by vision
     * @param robotStartingPosition the side of the field the robot is starting on
     * @return the backdrop ID, -1 if invalid arguments
     */
    public static int get(Direction spikeMarkPosition, StartingPositions robotStartingPosition) {
        switch (robotStartingPosition) {
            case STARTING_RED_LEFT:
            case STARTING_RED_RIGHT:
                switch (spikeMarkPosition) {
                    case LEFT:
                        return 4;
                    case FORWARD:
                        return 5;
                    case RIGHT:
                        return 6;
                    default:
                        return -1;
                }
            case STARTING_BLUE_LEFT:
            case STARTING_BLUE_RIGHT:
                switch (spikeMarkPosition) {
                    case LEFT:
                        return 1;
                    case FORWARD:
                        return 2;
                    case RIGHT:
                        return 3;
                    default:
                        return -1;
                }
            default:
                return -1;
        }
    }
}
