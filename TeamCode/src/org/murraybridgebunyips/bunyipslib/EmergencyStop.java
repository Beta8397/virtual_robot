package org.murraybridgebunyips.bunyipslib;

/**
 * Custom exception to be thrown when BunyipsLib should end the OpMode following a critical error.
 * This ensures ErrorUtil will be called but also allows the OpMode to be ended immediately.
 *
 * @author Lucas Bubner, 2024
 */
public class EmergencyStop extends RuntimeException {
    public EmergencyStop(String message) {
        super(message);
    }
}
