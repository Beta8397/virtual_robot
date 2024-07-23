/*
 * Copyright (c) 2022 DigitalChickenLabs
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

// Modified (Markedly) by Team Beta 8397 for use in the virtual_robot simulator

package com.qualcomm.hardware.digitalchickenlabs;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import java.util.Arrays;

public class OctoQuadImpl implements OctoQuad {

    private DcMotorEx[] encoders = new DcMotorEx[NUM_ENCODERS];

    private int[] velocitySampleIntervalsMS = new int[]{20, 20, 20, 20, 20, 20, 20, 20};

    private ChannelPulseWidthParams[] pulseWidthParams = new ChannelPulseWidthParams[]{
            new ChannelPulseWidthParams(10, 100),
            new ChannelPulseWidthParams(10, 100),
            new ChannelPulseWidthParams(10, 100),
            new ChannelPulseWidthParams(10, 100),
            new ChannelPulseWidthParams(10, 100),
            new ChannelPulseWidthParams(10, 100),
            new ChannelPulseWidthParams(10, 100),
            new ChannelPulseWidthParams(10, 100)
    };

    private ChannelBankConfig channelBankConfig = ChannelBankConfig.ALL_QUADRATURE;

    private I2cRecoveryMode i2cRecoveryMode = I2cRecoveryMode.NONE;

    public OctoQuadImpl(){};

    public OctoQuadImpl(DcMotorEx... encoders){
        for (int i=0; i<(encoders.length-1); i++){
            this.encoders[i] = encoders[i];
        }
    }

    public void setEncoder(int idx, DcMotorEx encoder){
        Range.throwIfRangeIsInvalid(idx, ENCODER_FIRST, ENCODER_LAST);
        encoders[idx] = encoder;
    }

    /**
     * Read a single position from the OctoQuad
     * Depending on the channel bank configuration, this may
     * either be quadrature step count, or pulse width.
     *
     * @param idx the index of the encoder to read
     * @return the position for the specified encoder
     */
    @Override
    public int readSinglePosition(int idx) {
        Range.throwIfRangeIsInvalid(idx, ENCODER_FIRST, ENCODER_LAST);
        if (encoders[idx] != null) {
            return encoders[idx].getCurrentPosition();
        } else {
            return 0;
        }
    }

    /**
     * Reads all positions from the OctoQuad, writing the data into
     * an existing int[] object. The previous values are destroyed.
     * Depending on the channel bank configuration, this may
     * either be quadrature step count, or pulse width.
     *
     * @param out the int[] object to fill with new data
     */
    @Override
    public void readAllPositions(int[] out) {
        if(out.length != NUM_ENCODERS)
        {
            throw new IllegalArgumentException("out.length != 8");
        }
        for (int i=0; i<NUM_ENCODERS; i++){
            if (encoders[i]!=null){
                out[i] = encoders[i].getCurrentPosition();
            } else {
                out[i] = 0;
            }
        }
    }

    /**
     * Reads all positions from the OctoQuad
     * Depending on the channel bank configuration, this may
     * either be quadrature step count, or pulse width.
     *
     * @return an int[] object with the new data
     */
    @Override
    public int[] readAllPositions() {
        int[] result = new int[NUM_ENCODERS];
        readAllPositions(result);
        return result;
    }

    /**
     * Read a selected range of encoders
     * Depending on the channel bank configuration, this may
     * either be quadrature step count, or pulse width.
     *
     * @param idxFirst the first encoder (inclusive)
     * @param idxLast  the last encoder (inclusive)
     * @return an array containing the requested encoder positions
     */
    @Override
    public int[] readPositionRange(int idxFirst, int idxLast) {
        Range.throwIfRangeIsInvalid(idxFirst, ENCODER_FIRST, ENCODER_LAST);
        Range.throwIfRangeIsInvalid(idxLast, ENCODER_FIRST, ENCODER_LAST);
        int length = idxLast - idxFirst + 1;
        int[] result= new int[length];
        for (int i=0; i<length; i++){
            result[i] = encoders[i] != null? encoders[idxFirst+i].getCurrentPosition() : 0;
        }
        return result;
    }

    /**
     * Reset multiple encoders in the OctoQuad firmware in one command
     *
     * @param resets the encoders to be reset
     */
    @Override
    public void resetMultiplePositions(boolean[] resets) {
        if(resets.length != NUM_ENCODERS)
        {
            throw new IllegalArgumentException("resets.length != 8");
        }
        for (int i=0; i<NUM_ENCODERS; i++){
            if (encoders[i]!=null) {
                DcMotor.RunMode prevMode = encoders[i].getMode();
                encoders[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                encoders[i].setMode(prevMode);
            }
        }
    }

    /**
     * Reset multiple encoders in the OctoQuad firmware in one command
     *
     * @param indices the indices of the encoders to reset
     */
    @Override
    public void resetMultiplePositions(int... indices) {
        for(int idx : indices)
        {
            Range.throwIfRangeIsInvalid(idx, ENCODER_FIRST, ENCODER_LAST);
        }
        for (int i=0; i<indices.length; i++){
            for (int idx : indices){
                if (encoders[idx] != null) {
                    DcMotor.RunMode prevMode = encoders[idx].getMode();
                    encoders[idx].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    encoders[idx].setMode(prevMode);
                }
            }
        }
    }

    /**
     * Set the direction for all encoders
     * This parameter will NOT be retained across power cycles, unless
     * you call {@link #saveParametersToFlash()} ()}
     *
     * @param reverse 8-length direction array
     */
    @Override
    public void setAllEncoderDirections(boolean[] reverse) {
        if(reverse.length != NUM_ENCODERS) {
            throw new IllegalArgumentException("reverse.length != 8");
        }
        for (int i=0; i<NUM_ENCODERS; i++){
            if (encoders[i] != null) {
                encoders[i].setDirection(reverse[i]? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
            }
        }
    }

    /**
     * Read a single velocity from the OctoQuad
     * NOTE: if using an absolute pulse width encoder, in order to get sane
     * velocity data, you must set the channel min/max pulse width parameter.
     *
     * @param idx the index of the encoder to read
     * @return the velocity for the specified encoder
     */
    @Override
    public short readSingleVelocity(int idx) {
        Range.throwIfRangeIsInvalid(idx, ENCODER_FIRST, ENCODER_LAST);
        if (encoders[idx] != null) {
            return (short)(encoders[idx].getVelocity());
        } else {
            return 0;
        }
    }

    /**
     * Reads all velocities from the OctoQuad, writing the data into
     * an existing short[] object. The previous values are destroyed.
     * NOTE: if using an absolute pulse width encoder, in order to get sane
     * velocity data, you must set the channel min/max pulse width parameter.
     *
     * @param out the short[] object to fill with new data
     */
    @Override
    public void readAllVelocities(short[] out) {
        if(out.length != NUM_ENCODERS) {
            throw new IllegalArgumentException("out.length != 8");
        }
        for (int i=0; i<NUM_ENCODERS; i++){
            if (encoders[i] != null) {
                out[i] = (short)(encoders[i].getVelocity());
            } else {
                out[i] = 0;
            }
        }
    }

    /**
     * Reads all velocities from the OctoQuad
     * NOTE: if using an absolute pulse width encoder, in order to get sane
     * velocity data, you must set the channel min/max pulse width parameter.
     *
     * @return a short[] object with the new data
     */
    @Override
    public short[] readAllVelocities() {
        short[] result = new short[NUM_ENCODERS];
        readAllVelocities(result);
        return result;
    }

    /**
     * Read a selected range of encoder velocities
     * NOTE: if using an absolute pulse width encoder, in order to get sane
     * velocity data, you must set the channel min/max pulse width parameter.
     *
     * @param idxFirst the first encoder (inclusive)
     * @param idxLast  the last encoder (inclusive)
     * @return an array containing the requested velocities
     */
    @Override
    public short[] readVelocityRange(int idxFirst, int idxLast) {
        Range.throwIfRangeIsInvalid(idxFirst, ENCODER_FIRST, ENCODER_LAST);
        Range.throwIfRangeIsInvalid(idxLast, ENCODER_FIRST, ENCODER_LAST);
        int length = idxLast - idxFirst + 1;
        short[] result= new short[length];
        for (int i=0; i<length; i++){
            result[i] = encoders[i] != null? (short)(encoders[idxFirst+i].getVelocity()) : 0;
        }
        return result;
    }

    /**
     * Reads all encoder data from the OctoQuad, writing the data into
     * an existing {@link EncoderDataBlock} object. The previous values are destroyed.
     *
     * @param out the {@link EncoderDataBlock} object to fill with new data
     */
    @Override
    public void readAllEncoderData(EncoderDataBlock out) {
        if(out.positions.length != NUM_ENCODERS) {
            throw new IllegalArgumentException("out.counts.length != 8");
        }

        if(out.velocities.length != NUM_ENCODERS) {
            throw new IllegalArgumentException("out.velocities.length != 8");
        }
        readAllPositions(out.positions);
        readAllVelocities(out.velocities);
    }

    /**
     * Reads all encoder data from the OctoQuad
     *
     * @return a {@link EncoderDataBlock} object with the new data
     */
    @Override
    public EncoderDataBlock readAllEncoderData() {
        EncoderDataBlock result = new EncoderDataBlock();
        result.positions = new int[NUM_ENCODERS];
        result.velocities = new short[NUM_ENCODERS];
        readAllEncoderData(result);
        return result;
    }

    /**
     * Set the velocity sample intervals for all encoders
     * This parameter will NOT be retained across power cycles, unless
     * you call {@link #saveParametersToFlash()} ()}
     *
     * @param intvlms the sample intervals in milliseconds
     */
    @Override
    public void setAllVelocitySampleIntervals(int[] intvlms) {
        if(intvlms.length != NUM_ENCODERS) {
            throw new IllegalArgumentException("intvls.length != 8");
        }

        for (int i=0; i<NUM_ENCODERS; i++){
            velocitySampleIntervalsMS[i] = intvlms[i];
        }
    }

    /**
     * Reads all velocity sample intervals from the OctoQuad
     *
     * @return all velocity sample intervals from the OctoQuad
     */
    @Override
    public int[] getAllVelocitySampleIntervals() {
        int[] result = Arrays.copyOf(velocitySampleIntervalsMS, NUM_ENCODERS);
        return result;
    }

    /**
     * Configure the minimum/maximum pulse width reported by an absolute encoder
     * which is connected to a given channel, to allow the ability to provide
     * accurate velocity data.
     * These parameters will NOT be retained across power cycles, unless
     * you call {@link #saveParametersToFlash()} ()}
     *
     * @param idx    the channel in question
     * @param params minimum/maximum pulse width
     */
    @Override
    public void setSingleChannelPulseWidthParams(int idx, ChannelPulseWidthParams params) {
        Range.throwIfRangeIsInvalid(idx, ENCODER_FIRST, ENCODER_LAST);
        Range.throwIfRangeIsInvalid(params.min_length_us, MIN_PULSE_WIDTH_US, MAX_PULSE_WIDTH_US);
        Range.throwIfRangeIsInvalid(params.max_length_us, MIN_PULSE_WIDTH_US, MAX_PULSE_WIDTH_US);

        if(params.max_length_us <= params.min_length_us) {
            throw new RuntimeException("params.max_length_us <= params.min_length_us");
        }

        pulseWidthParams[idx] = params;
    }

    /**
     * Queries the OctoQuad to determine the currently set minimum/maxiumum pulse
     * width for an encoder channel, to allow sane velocity data.
     *
     * @param idx the channel in question
     * @return minimum/maximum pulse width
     */
    @Override
    public ChannelPulseWidthParams getSingleChannelPulseWidthParams(int idx) {
        Range.throwIfRangeIsInvalid(idx, ENCODER_FIRST, ENCODER_LAST);
        return new ChannelPulseWidthParams(pulseWidthParams[idx].min_length_us, pulseWidthParams[idx].max_length_us);
    }

    /**
     * Get the firmware version running on the OctoQuad
     *
     * @return the firmware version running on the OctoQuad
     */
    @Override
    public String getFirmwareVersionString() {
        return getFirmwareVersion().toString();
    }

    /**
     * Reset a single encoder in the OctoQuad firmware
     *
     * @param idx the index of the encoder to reset
     */
    @Override
    public void resetSinglePosition(int idx) {
        Range.throwIfRangeIsInvalid(idx, ENCODER_FIRST, ENCODER_LAST);
        if (encoders[idx] != null) {
            DcMotor.RunMode prevMode = encoders[idx].getMode();
            encoders[idx].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            encoders[idx].setMode(prevMode);
        }
    }

    /**
     * Reset all encoder counts in the OctoQuad firmware
     */
    @Override
    public void resetAllPositions() {
        for (int i=0; i<NUM_ENCODERS; i++){
            if (encoders[i] != null) {
                DcMotor.RunMode prevMode = encoders[i].getMode();
                encoders[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                encoders[i].setMode(prevMode);
            }
        }
    }

    /**
     * Set the direction for a single encoder
     * This parameter will NOT be retained across power cycles, unless
     * you call {@link #saveParametersToFlash()} ()}
     *
     * @param idx the index of the encoder
     * @param dir direction
     */
    @Override
    public void setSingleEncoderDirection(int idx, EncoderDirection dir) {
        Range.throwIfRangeIsInvalid(idx, ENCODER_FIRST, ENCODER_LAST);
        if (encoders[idx] == null) return;
        if(dir == EncoderDirection.REVERSE)
        {
            encoders[idx].setDirection(DcMotorSimple.Direction.REVERSE);
        }
        else
        {
            encoders[idx].setDirection(DcMotorSimple.Direction.FORWARD);
        }
    }

    /**
     * Get the direction for a single encoder
     *
     * @param idx the index of the encoder
     * @return direction of the encoder in question
     */
    @Override
    public EncoderDirection getSingleEncoderDirection(int idx) {
        Range.throwIfRangeIsInvalid(idx, ENCODER_FIRST, ENCODER_LAST);
        if (encoders[idx] == null || encoders[idx].getDirection() == DcMotorSimple.Direction.FORWARD){
            return EncoderDirection.FORWARD;
        } else {
            return EncoderDirection.REVERSE;
        }
    }

    /**
     * Set the velocity sample interval for a single encoder
     * This parameter will NOT be retained across power cycles, unless
     * you call {@link #saveParametersToFlash()} ()}
     *
     * @param idx     the index of the encoder in question
     * @param intvlms the sample interval in milliseconds
     */
    @Override
    public void setSingleVelocitySampleInterval(int idx, int intvlms) {
        Range.throwIfRangeIsInvalid(idx, ENCODER_FIRST, ENCODER_LAST);
        Range.throwIfRangeIsInvalid(intvlms, MIN_VELOCITY_MEASUREMENT_INTERVAL_MS, MAX_VELOCITY_MEASUREMENT_INTERVAL_MS);
        velocitySampleIntervalsMS[idx] = intvlms;
    }

    /**
     * Set the velocity sample interval for all encoders
     * This parameter will NOT be retained across power cycles, unless
     * you call {@link #saveParametersToFlash()} ()}
     *
     * @param intvlms the sample interval in milliseconds
     */
    @Override
    public void setAllVelocitySampleIntervals(int intvlms) {
        Range.throwIfRangeIsInvalid(intvlms, MIN_VELOCITY_MEASUREMENT_INTERVAL_MS, MAX_VELOCITY_MEASUREMENT_INTERVAL_MS);
        for (int i=0; i<NUM_ENCODERS; i++){
            velocitySampleIntervalsMS[i] = intvlms;
        }
    }

    /**
     * Read a single velocity sample interval
     *
     * @param idx the index of the encoder in question
     * @return the velocity sample interval
     */
    @Override
    public int getSingleVelocitySampleInterval(int idx) {
        Range.throwIfRangeIsInvalid(idx, ENCODER_FIRST, ENCODER_LAST);
        return velocitySampleIntervalsMS[idx];
    }

    /**
     * Configure the minimum/maximum pulse width reported by an absolute encoder
     * which is connected to a given channel, to allow the ability to provide
     * accurate velocity data.
     * These parameters will NOT be retained across power cycles, unless
     * you call {@link #saveParametersToFlash()} ()}
     *
     * @param idx           the channel in question
     * @param min_length_us minimum pulse width
     * @param max_length_us maximum pulse width
     */
    @Override
    public void setSingleChannelPulseWidthParams(int idx, int min_length_us, int max_length_us) {
        setSingleChannelPulseWidthParams(idx, new ChannelPulseWidthParams(min_length_us, max_length_us));
    }

    /**
     * Run the firmware's internal reset routine
     */
    @Override
    public void resetEverything() {
        resetAllPositions();
        velocitySampleIntervalsMS = new int[]{20, 20, 20, 20, 20, 20, 20, 20};

        pulseWidthParams = new ChannelPulseWidthParams[]{
                new ChannelPulseWidthParams(10, 100),
                new ChannelPulseWidthParams(10, 100),
                new ChannelPulseWidthParams(10, 100),
                new ChannelPulseWidthParams(10, 100),
                new ChannelPulseWidthParams(10, 100),
                new ChannelPulseWidthParams(10, 100),
                new ChannelPulseWidthParams(10, 100),
                new ChannelPulseWidthParams(10, 100)
        };

        channelBankConfig = ChannelBankConfig.ALL_QUADRATURE;
        i2cRecoveryMode = I2cRecoveryMode.NONE;
    }

    /**
     * Configures the OctoQuad's channel banks
     * This parameter will NOT be retained across power cycles, unless
     * you call {@link #saveParametersToFlash()} ()}
     *
     * @param config the channel bank configuration to use
     */
    @Override
    public void setChannelBankConfig(ChannelBankConfig config) {
        channelBankConfig = config;
    }

    /**
     * Queries the OctoQuad to determine the current channel bank configuration
     *
     * @return the current channel bank configuration
     */
    @Override
    public ChannelBankConfig getChannelBankConfig() {
        return channelBankConfig;
    }

    /**
     * Configures the OctoQuad to use the specified I2C recovery mode.
     * This parameter will NOT be retained across power cycles, unless
     * you call {@link #saveParametersToFlash()} ()}
     *
     * @param mode the recovery mode to use
     */
    @Override
    public void setI2cRecoveryMode(I2cRecoveryMode mode) {
        i2cRecoveryMode = mode;
    }

    /**
     * Queries the OctoQuad to determine the currently configured I2C recovery mode
     *
     * @return the currently configured I2C recovery mode
     */
    @Override
    public I2cRecoveryMode getI2cRecoveryMode() {
        return i2cRecoveryMode;
    }

    /**
     * Stores the current state of parameters to flash, to be applied at next boot
     */
    @Override
    public void saveParametersToFlash() {

    }
}
