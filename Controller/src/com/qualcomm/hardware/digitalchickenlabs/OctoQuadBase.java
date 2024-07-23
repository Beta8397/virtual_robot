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
// Modified by Team Beta 8397 for use in the virtual_robot simulator

package com.qualcomm.hardware.digitalchickenlabs;

import com.qualcomm.robotcore.hardware.HardwareDevice;

public interface OctoQuadBase extends HardwareDevice {
    byte OCTOQUAD_CHIP_ID = 0x51;
    int SUPPORTED_FW_VERSION_MAJ = 2;
    int ENCODER_FIRST = 0;
    int ENCODER_LAST = 7;
    int NUM_ENCODERS = 8;
    int MIN_VELOCITY_MEASUREMENT_INTERVAL_MS = 1;
    int MAX_VELOCITY_MEASUREMENT_INTERVAL_MS = 255;
    int MIN_PULSE_WIDTH_US = 1;  //  The symbol for microseconds is μs, but is sometimes simplified to us.
    int MAX_PULSE_WIDTH_US = 0xFFFF;

    /**
     * Reads the CHIP_ID register of the OctoQuad
     * @return the value in the CHIP_ID register of the OctoQuad
     */
    default byte getChipId(){
        return 0;
    }

    /**
     * Get the firmware version running on the OctoQuad
     * @return the firmware version running on the OctoQuad
     */
    String getFirmwareVersionString();

    /**
     * Reset a single encoder in the OctoQuad firmware
     * @param idx the index of the encoder to reset
     */
    void resetSinglePosition(int idx);

    /**
     * Reset all encoder counts in the OctoQuad firmware
     */
    void resetAllPositions();

    enum EncoderDirection
    {
        FORWARD,
        REVERSE
    }

    /**
     * Set the direction for a single encoder
     * This parameter will NOT be retained across power cycles, unless
     * you call {@link #saveParametersToFlash()} ()}
     * @param idx the index of the encoder
     * @param dir direction
     */
    void setSingleEncoderDirection(int idx, EncoderDirection dir);

    /**
     * Get the direction for a single encoder
     * @param idx the index of the encoder
     * @return direction of the encoder in question
     */
    EncoderDirection getSingleEncoderDirection(int idx);

    /**
     * Set the velocity sample interval for a single encoder
     * This parameter will NOT be retained across power cycles, unless
     * you call {@link #saveParametersToFlash()} ()}
     * @param idx the index of the encoder in question
     * @param intvlms the sample interval in milliseconds
     */
    void setSingleVelocitySampleInterval(int idx, int intvlms);

    /**
     * Set the velocity sample interval for all encoders
     * This parameter will NOT be retained across power cycles, unless
     * you call {@link #saveParametersToFlash()} ()}
     * @param intvlms the sample interval in milliseconds
     */
    void setAllVelocitySampleIntervals(int intvlms);

    /**
     * Read a single velocity sample interval
     * @param idx the index of the encoder in question
     * @return the velocity sample interval
     */
    int getSingleVelocitySampleInterval(int idx);

    /**
     * Configure the minimum/maximum pulse width reported by an absolute encoder
     * which is connected to a given channel, to allow the ability to provide
     * accurate velocity data.
     * These parameters will NOT be retained across power cycles, unless
     * you call {@link #saveParametersToFlash()} ()}
     * @param idx the channel in question
     * @param min_length_us minimum pulse width
     * @param max_length_us maximum pulse width
     */
    void setSingleChannelPulseWidthParams(int idx, int min_length_us, int max_length_us);

    /**
     * Run the firmware's internal reset routine
     */
    void resetEverything();

    enum ChannelBankConfig
    {
        /**
         * Both channel banks are configured for Quadrature input
         */
        ALL_QUADRATURE(0),

        /**
         * Both channel banks are configured for pulse width input
         */
        ALL_PULSE_WIDTH(1),

        /**
         * Bank 1 (channels 0-3) is configured for Quadrature input;
         * Bank 2 (channels 4-7) is configured for pulse width input.
         */
        BANK1_QUADRATURE_BANK2_PULSE_WIDTH(2);

        public byte bVal;

        ChannelBankConfig(int bVal)
        {
            this.bVal = (byte) bVal;
        }
    }

    /**
     * Configures the OctoQuad's channel banks
     * This parameter will NOT be retained across power cycles, unless
     * you call {@link #saveParametersToFlash()} ()}
     * @param config the channel bank configuration to use
     */
    void setChannelBankConfig(ChannelBankConfig config);

    /**
     * Queries the OctoQuad to determine the current channel bank configuration
     * @return the current channel bank configuration
     */
    ChannelBankConfig getChannelBankConfig();

    enum I2cRecoveryMode
    {
        /**
         * Does not perform any active attempts to recover a wedged I2C bus
         */
        NONE(0),

        /**
         * The OctoQuad will reset its I2C peripheral if 50ms elapses between
         * byte transmissions or between bytes and start/stop conditions
         */
        MODE_1_PERIPH_RST_ON_FRAME_ERR(1),

        /**
         * Mode 1 actions + the OctoQuad will toggle the clock line briefly,
         * once, after 1500ms of no communications.
         */
        MODE_2_M1_PLUS_SCL_IDLE_ONESHOT_TGL(2);

        public byte bVal;

        I2cRecoveryMode(int bVal)
        {
            this.bVal = (byte) bVal;
        }
    }

    /**
     * Configures the OctoQuad to use the specified I2C recovery mode.
     * This parameter will NOT be retained across power cycles, unless
     * you call {@link #saveParametersToFlash()} ()}
     * @param mode the recovery mode to use
     */
    void setI2cRecoveryMode(I2cRecoveryMode mode);

    /**
     * Queries the OctoQuad to determine the currently configured I2C recovery mode
     * @return the currently configured I2C recovery mode
     */
    I2cRecoveryMode getI2cRecoveryMode();

    /**
     * Stores the current state of parameters to flash, to be applied at next boot
     */
    void saveParametersToFlash();
}
