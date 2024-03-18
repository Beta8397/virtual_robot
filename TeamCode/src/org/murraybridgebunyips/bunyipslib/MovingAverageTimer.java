package org.murraybridgebunyips.bunyipslib;

import androidx.annotation.NonNull;

/**
 * Time utilities for robot operation.
 * <a href="https://github.com/Paladins-of-St-Pauls/GameChangers/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/paladins/utils/MovingAverageTimer.java">Source</a>
 *
 * @author Shaun, 11/06/2017
 */
public class MovingAverageTimer {
    /**
     * The number of nanoseconds in a second.
     */
    public static final double NANOS_IN_SECONDS = 1000000000.0;
    /**
     * The number of nanoseconds in a millisecond.
     */
    public static final double NANOS_IN_MILLIS = 1000000.0;

    // A ring buffer is used to keep track of a moving average
    private final int ringBufferSize;
    private final long[] loopTimeRingBuffer;
    private final double resolution;
    private final String avgFormatStr;
    private final String toStringFormatStr;
    private int ringBufferIndex;
    private long loopCount;
    private long movingTotal;
    private long runningTotal;
    private long previousTime;
    private double movingAverage;
    private double minMovingAverage = Double.MAX_VALUE;
    private double maxMovingAverage = Double.MIN_VALUE;
    private double average;
    private double minAverage = Double.MAX_VALUE;
    private double maxAverage = Double.MIN_VALUE;
    private double minLoopTime = Double.MAX_VALUE;
    private double maxLoopTime = Double.MIN_VALUE;

    /**
     * Create a new MovingAverageTimer with a ring buffer size of 100 and a resolution of milliseconds.
     */
    public MovingAverageTimer() {
        this(100, Resolution.MILLISECONDS);
    }

    /**
     * Create a new MovingAverageTimer with the specified buffer size and a resolution of milliseconds.
     *
     * @param bufSize the size of the ring buffer
     */
    public MovingAverageTimer(int bufSize) {
        this(bufSize, Resolution.MILLISECONDS);
    }

    /**
     * Create a new MovingAverageTimer with the specified buffer size and resolution.
     *
     * @param bufSize    the size of the ring buffer
     * @param resolution the resolution of the timer
     */
    public MovingAverageTimer(int bufSize, Resolution resolution) {
        reset();
        ringBufferSize = bufSize;
        loopTimeRingBuffer = new long[ringBufferSize];

        String hdr = String.format("\n%-12s%-12s%-12s%-12s", "Loops", "TotalTime", "MovAvg", "Avg");

        switch (resolution) {
            case SECONDS:
                this.resolution = NANOS_IN_SECONDS;
                avgFormatStr = "%3.3fs";
                toStringFormatStr = hdr + " seconds\n%-12d%-12.3f%-12.3f%-12.3f\n min        %-12.3f%-12.3f%-12.3f\n max        %-12.3f%-12.3f%-12.3f";
                break;
            case MILLISECONDS:
            default:
                this.resolution = NANOS_IN_MILLIS;
                avgFormatStr = "%3.3fms";
                toStringFormatStr = hdr + "msecs\n%-12d%-12.3f%-12.3f%-12.3f\n min        %-12.3f%-12.3f%-12.3f\n max        %-12.3f%-12.3f%-12.3f";
                break;
        }
    }

    /**
     * Reset the timer.
     */
    public void reset() {
        loopCount = 0;
        previousTime = System.nanoTime();
        movingTotal = 0;
        runningTotal = 0;
        movingAverage = 0;
        average = 0.0;
    }

    /**
     * Update the timer. Should be called once per loop.
     */
    public void update() {
        long now = System.nanoTime();
        long loopTime = now - previousTime;
        previousTime = now;

        if (loopCount > 0) {
            minLoopTime = Math.min(minLoopTime, loopTime);
            maxLoopTime = Math.max(maxLoopTime, loopTime);
        }

        // Adjust the running total
        movingTotal = movingTotal - loopTimeRingBuffer[ringBufferIndex] + loopTime;
        runningTotal = runningTotal + loopTime;

        // Add the new value
        loopTimeRingBuffer[ringBufferIndex] = loopTime;

        // wrap the current index
        ringBufferIndex = (ringBufferIndex + 1) % ringBufferSize;

        loopCount += 1;

        if (loopCount < ringBufferSize) {
            if (loopCount == 0) {
                movingAverage = 0.0;
            } else {
                movingAverage = (double) movingTotal / loopCount / resolution;
            }
            // Temporarily fill the min/max movingAverage
            minMovingAverage = Math.min(minMovingAverage, movingAverage);
            maxMovingAverage = Math.max(maxMovingAverage, movingAverage);

        } else {
            movingAverage = (double) movingTotal / ringBufferSize / resolution;

            // Reset the min/max movingAverage values the each time the buffer is filled
            if (ringBufferIndex == 0) {
                minMovingAverage = movingAverage;
                maxMovingAverage = movingAverage;
            } else {
                minMovingAverage = Math.min(minMovingAverage, movingAverage);
                maxMovingAverage = Math.max(maxMovingAverage, movingAverage);
            }
        }

        average = (double) runningTotal / loopCount / resolution;
        minAverage = Math.min(minAverage, average);
        maxAverage = Math.max(maxAverage, average);
    }

    /**
     * Get the number of loops that have been counted.
     *
     * @return the number of loops
     */
    public long count() {
        return loopCount;
    }

    /**
     * @return the number of loops per second
     */
    public double loopsSec() {
        return loopCount / (runningTotal / NANOS_IN_SECONDS);
    }

    /**
     * @return the moving average loop time
     */
    public double movingAverage() {
        return movingAverage;
    }

    /**
     * @return the minimum moving average loop time
     */
    public double minMovingAverage() {
        return minMovingAverage;
    }

    /**
     * @return the maximum moving average loop time
     */
    public double maxMovingAverage() {
        return maxMovingAverage;
    }

    /**
     * @return A string representation of the moving average loop time
     */
    public String movingAverageString() {
        return String.format(avgFormatStr, movingAverage);
    }

    /**
     * @return the average loop time
     */
    public double average() {
        return average;
    }

    /**
     * @return the minimum average loop time
     */
    public double minAverage() {
        return minAverage;
    }

    /**
     * @return the maximum average loop time
     */
    public double maxAverage() {
        return maxAverage;
    }

    /**
     * @return the minimum loop time
     */
    public double minLoopTime() {
        return minLoopTime / resolution;
    }

    /**
     * @return the maximum loop time
     */
    public double maxLoopTime() {
        return maxLoopTime / resolution;
    }

    /**
     * @return the total time elapsed
     */
    public double elapsedTime() {
        return runningTotal / resolution;
    }

    /**
     * @return A string representation of the average loop time
     */
    public String averageString() {
        return String.format(avgFormatStr, average);
    }

    @NonNull
    @Override
    public String toString() {
        return String.format(toStringFormatStr, loopCount, elapsedTime(), movingAverage, average, minLoopTime(), minMovingAverage, minAverage, maxLoopTime(), maxMovingAverage, maxAverage);
    }

    /**
     * The resolution of the timer.
     */
    public enum Resolution {
        /**
         * Report times in seconds.
         */
        SECONDS,
        /**
         * Report times in milliseconds.
         */
        MILLISECONDS
    }
}
