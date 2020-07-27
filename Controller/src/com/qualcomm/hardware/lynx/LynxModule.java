/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/*
This file has been modified (severely) by FTC Team Beta for use in the Virtual Robot simulator.

It is now essentially a dummy class, included for compatibility with op modes that use bulk caching.
Caching modes set in this class have NO EFFECT ON SIMULATOR FUNCTION.
 */

package com.qualcomm.hardware.lynx;

import com.qualcomm.robotcore.hardware.HardwareDevice;

public class LynxModule implements HardwareDevice {

    BulkCachingMode bulkCachingMode = BulkCachingMode.OFF;

    /**
     * Bulk caching mode that controls the behavior of certain read commands in the actual FTC SDK.
     * These modes have no effect on simulator behavior.
     */
    public enum BulkCachingMode
    {
        /**
         * Never replace commands with cached bulk read results.
         */
        OFF,
        /**
         * Replace eligible commands with the results of a cached bulk read. The cache must be
         * manually cleared with {@link #clearBulkCache()} before another bulk read is issued. This
         * should only be used by advanced users. A common pattern is placing a single
         * {@link #clearBulkCache()} call at the start of every hardware loop.
         */
        MANUAL,
        /**
         * Same as {@link #MANUAL} except the cache is also cleared automatically when the same
         * command is issued twice. This mode is intended for beginning users that want to benefit
         * from bulk reads without explicit cache-handling code.
         */
        AUTO
    }

    /**
     * Returns the current bulk caching mode.
     * @return current bulk caching mode
     */
    public BulkCachingMode getBulkCachingMode()
    {
        return bulkCachingMode;
    }

    /**
     * Sets the bulk caching mode and clears the cache if necessary.
     * @param mode new bulk caching mode
     */
    public void setBulkCachingMode(BulkCachingMode mode)
    {
        if (bulkCachingMode == BulkCachingMode.OFF && mode != BulkCachingMode.OFF)
        {
            clearBulkCache();
        }
        bulkCachingMode = mode;
    }

    /**
     * Clears the bulk read cache. DUMMY METHOD, included only for compatibility with op modes that
     * use bulk caching.
     */
    public void clearBulkCache() {}


}
