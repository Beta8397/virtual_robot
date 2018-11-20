package hardware;

/**
 * DCMotor is an abridged version of the FTC DCMotor interface.
 */
public interface DCMotor {
    public enum Direction {FORWARD, REVERSE}

    public enum RunMode {RUN_TO_POSITION, RUN_USING_ENCODER, RUN_WITHOUT_ENCODER, STOP_AND_RESET_ENCODER}

    /**
     * Set operation mode of the motor.
     * @param mode Note: RUN_USING_ENCODER and RUN_WITHOUT_ENCODER will behave the same in this simulator. For real
     *             robot programming, they will behave very differently, and it's essential to choose the appropriate
     *             mode. RUN_TO_POSITION is not implemented in the simulator. Setting mode to STOP_AND_RESET_ENCODER
     *             will set the power to zero and zero the encoder. To run the motor again, the mode must be set to
     *             either RUN_USING_ENCODER or RUN_WITHOUT_ENCODER.
     */
    public void setMode(RunMode mode);

    /**
     * Get the operation mode of the motor.
     * @return
     */
    public RunMode getMode();

    /**
     * Set the DIRECTION of the motor (in this simulator, you would typically use REVERSE for the left motor and
     * FORWARD for the right motor).
     * @param direction
     */
    public void setDirection(Direction direction);

    /**
     * Get the direction of the motor.
     * @return motor direction
     */
    public Direction getDirection();

    /**
     * Set the motor power.
     * @param power (-1 to +1)
     */
    public void setPower(double power);

    /**
     * Get the motor power
     * @return motor power
     */
    public double getPower();

    /**
     * Get current motor position (i.e., encoder ticks)
     * @return encoder ticks
     */
    public int getCurrentPosition();

}
