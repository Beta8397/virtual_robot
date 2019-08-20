package com.qualcomm.robotcore.hardware;

import com.qualcomm.robotcore.hardware.HardwareDevice;

/**
 * DcMotor is an abridged version of the FTC DcMotor interface.
 */
public interface DcMotor extends DcMotorSimple {

    /**
     * Enum of operation modes available for DcMotor.
     * Note: RUN_USING_ENCODER and RUN_WITHOUT_ENCODER will behave the same in this simulator. For real
     *     robot programming, they will behave very differently, and it's essential to choose the appropriate
     *     mode. RUN_TO_POSITION is now implemented in the simulator. Setting mode to STOP_AND_RESET_ENCODER
     *     will set the power to zero and zero the encoder. To run the motor again, the mode must be set to
     *     either RUN_USING_ENCODER, RUN_WITHOUT_ENCODER, or RUN_TO_POSITION.
     */
    public enum RunMode {RUN_TO_POSITION, RUN_USING_ENCODER, RUN_WITHOUT_ENCODER, STOP_AND_RESET_ENCODER}

    /**
     * Set operation mode of the motor.
     */
    public void setMode(RunMode mode);

    /**
     * Get the operation mode of the motor.
     * @return Operation Mode
     */
    public RunMode getMode();

    /**
     * Get current motor position (i.e., encoder ticks)
     * @return encoder ticks
     */
    public int getCurrentPosition();

    /**
     * Set the target position (in encoder ticks) for RUN_TO_POSITION mode
     * @param pos target position
     */
    public void setTargetPosition(int pos);

    /**
     * Get the target position (in encoder ticks) for RUN_TO_POSITION mode
     * @return target position
     */
    public int getTargetPosition();

    /**
     * Indicates whether motor is actively approaching a target position in RUN_TO_POSITION mode
     * @return True if actively approaching a target
     */
    public boolean isBusy();

}
