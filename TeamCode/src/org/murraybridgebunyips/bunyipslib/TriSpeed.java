package org.murraybridgebunyips.bunyipslib;

/**
 * Tri-speed configuration for gamepad inputs.
 * This is an update-less component that does not need active polling with BunyipsOpMode.
 *
 * @author Lucas Bubner, 2023
 */
public class TriSpeed {
    private Speed speed;

    public TriSpeed(Speed initialSpeed) {
        speed = initialSpeed;
    }

    public Speed getSpeed() {
        return speed;
    }

    /**
     * Increment speed
     */
    public void increment() {
        if (speed == Speed.SLOW) {
            speed = Speed.NORMAL;
        } else if (speed == Speed.NORMAL) {
            speed = Speed.FAST;
        }
    }

    /**
     * Decrement speed
     */
    public void decrement() {
        if (speed == Speed.FAST) {
            speed = Speed.NORMAL;
        } else if (speed == Speed.NORMAL) {
            speed = Speed.SLOW;
        }
    }

    /**
     * Multiplier for the current speed that can be used for scaling
     *
     * @return multiplier for the current speed
     */
    public double getMultiplier() {
        if (speed == Speed.SLOW) {
            return 0.25;
        } else if (speed == Speed.NORMAL) {
            return 0.5;
        } else {
            return 1.0;
        }
    }

    public enum Speed {
        SLOW,
        NORMAL,
        FAST
    }
}
