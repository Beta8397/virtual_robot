package org.murraybridgebunyips.bunyipslib.roadrunner.util;

import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.murraybridgebunyips.bunyipslib.Dbg;
import org.murraybridgebunyips.bunyipslib.RobotConfig;

/**
 * Wraps a motor instance to provide corrected velocity counts and allow reversing independently of the corresponding
 * slot's motor direction
 */
public class Encoder {
    private static final int CPS_STEP = 0x10000;
    private DcMotorEx motor;
    private NanoClock clock;
    private double[] velocityEstimates;
    private Direction direction;
    private int lastPosition;
    private int velocityEstimateIdx;
    private double lastUpdateTime;

    /**
     * Constructs a new Encoder instance.
     *
     * @param motor the motor to wrap
     * @param clock the clock to use
     */
    public Encoder(DcMotorEx motor, NanoClock clock) {
        this.motor = motor;
        this.clock = clock;

        direction = Direction.FORWARD;

        lastPosition = 0;
        velocityEstimates = new double[3];
        lastUpdateTime = clock.seconds();
    }

    /**
     * Constructs a new Encoder instance.
     *
     * @param motor the motor to wrap
     */
    public Encoder(DcMotorEx motor) {
        this(motor, NanoClock.system());
    }

    /**
     * Constructs a new Encoder instance.
     *
     * @param hardwareMapName the name of the motor in the hardware map
     * @param configInstance  the RobotConfig instance for easy instantiation
     * @param clock           the clock to use
     */
    public Encoder(String hardwareMapName, RobotConfig configInstance, NanoClock clock) {
        DcMotorEx motor = configInstance.getHardware(hardwareMapName, DcMotorEx.class);
        if (motor == null) {
            // Instantiation will still continue, NullPointerExceptions are bound to happen beyond this point
            // Will need to check for null with isNull(), or to simply fix the problem (checking the instance itself for null will not work)
            Dbg.warn("Encoder with name `%` failed to instantiate due to null motor. Ensure to check encoder.isNull() before using this device, or correcting the misconfiguration.", hardwareMapName);
            return;
        }

        this.motor = motor;
        this.clock = clock;

        direction = Direction.FORWARD;

        lastPosition = 0;
        velocityEstimates = new double[3];
        lastUpdateTime = clock.seconds();
    }

    /**
     * Constructs a new Encoder instance.
     *
     * @param hardwareMapName the name of the motor in the hardware map
     * @param configInstance  the RobotConfig instance for easy instantiation
     */
    public Encoder(String hardwareMapName, RobotConfig configInstance) {
        this(hardwareMapName, configInstance, NanoClock.system());
    }

    private static double inverseOverflow(double input, double estimate) {
        // convert to uint16
        int real = (int) input & 0xffff;
        // initial, modulo-based correction: it can recover the remainder of 5 of the upper 16 bits
        // because the velocity is always a multiple of 20 cps due to Expansion Hub's 50ms measurement window
        real += ((real % 20) / 4) * CPS_STEP;
        // estimate-based correction: it finds the nearest multiple of 5 to correct the upper bits by
        real += Math.round((estimate - real) / (5 * CPS_STEP)) * 5 * CPS_STEP;
        return real;
    }

    /**
     * Checks if the underlying motor for the encoder is null
     * This is important to check as the encoder will still instantiate, but will not function correctly
     * NullSafety.assertComponentArgs() will ensure to check for null motors on Encoders.
     */
    public boolean isNull() {
        return motor == null;
    }

    public Direction getDirection() {
        return direction;
    }

    /**
     * Allows you to set the direction of the counts and velocity without modifying the motor's direction state
     *
     * @param direction either reverse or forward depending on if encoder counts should be negated
     */
    public void setDirection(Direction direction) {
        this.direction = direction;
    }

    private int getMultiplier() {
        return direction.getMultiplier() * (motor.getDirection() == DcMotorSimple.Direction.FORWARD ? 1 : -1);
    }

    /**
     * Gets the position from the underlying motor and adjusts for the set direction.
     * Additionally, this method updates the velocity estimates used for compensated velocity
     *
     * @return encoder position
     */
    public int getCurrentPosition() {
        int multiplier = getMultiplier();
        int currentPosition = motor.getCurrentPosition() * multiplier;
        if (currentPosition != lastPosition) {
            double currentTime = clock.seconds();
            double dt = currentTime - lastUpdateTime;
            velocityEstimates[velocityEstimateIdx] = (currentPosition - lastPosition) / dt;
            velocityEstimateIdx = (velocityEstimateIdx + 1) % 3;
            lastPosition = currentPosition;
            lastUpdateTime = currentTime;
        }
        return currentPosition;
    }

    /**
     * Gets the velocity directly from the underlying motor and compensates for the direction
     * See {@link #getCorrectedVelocity} for high (>2^15) counts per second velocities (such as on REV Through Bore)
     *
     * @return raw velocity
     */
    public double getRawVelocity() {
        int multiplier = getMultiplier();
        return motor.getVelocity() * multiplier;
    }

    /**
     * Uses velocity estimates gathered in {@link #getCurrentPosition} to estimate the upper bits of velocity
     * that are lost in overflow due to velocity being transmitted as 16 bits.
     * CAVEAT: must regularly call {@link #getCurrentPosition} for the compensation to work correctly.
     *
     * @return corrected velocity
     */
    public double getCorrectedVelocity() {
        double median = velocityEstimates[0] > velocityEstimates[1]
                ? Math.max(velocityEstimates[1], Math.min(velocityEstimates[0], velocityEstimates[2]))
                : Math.max(velocityEstimates[0], Math.min(velocityEstimates[1], velocityEstimates[2]));
        return inverseOverflow(getRawVelocity(), median);
    }

    /**
     * Direction of encoder counts
     */
    public enum Direction {
        /**
         * Forward direction
         */
        FORWARD(1),
        /**
         * Reverse direction
         */
        REVERSE(-1);

        private final int multiplier;

        Direction(int multiplier) {
            this.multiplier = multiplier;
        }

        public int getMultiplier() {
            return multiplier;
        }
    }
}
