package org.murraybridgebunyips.bunyipslib;

/**
 * Time utilities for robot operation.
 * <a href="https://github.com/Paladins-of-St-Pauls/GameChangers/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/paladins/utils/MovingAverageTimer.java">Source</a>
 *
 * @author Shaun, 11/06/2017
 */
public class MovingAverageTimer {
    public static final double SECOND_IN_NANO = 1000000000.0;
    public static final double MILLIS_IN_NANO = 1000000.0;

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

    public MovingAverageTimer() {
        this(100, Resolution.MILLISECONDS);
    }

    public MovingAverageTimer(int bufSize) {
        this(bufSize, Resolution.MILLISECONDS);
    }

    public MovingAverageTimer(int bufSize, Resolution resolution) {
        reset();
        ringBufferSize = bufSize;
        loopTimeRingBuffer = new long[ringBufferSize];

        String hdr = String.format("\n%-12s%-12s%-12s%-12s", "Loops", "TotalTime", "MovAvg", "Avg");

        switch (resolution) {
            case SECONDS:
                this.resolution = SECOND_IN_NANO;
                avgFormatStr = "%3.3fs";
                toStringFormatStr = hdr + " seconds\n%-12d%-12.3f%-12.3f%-12.3f\n min        %-12.3f%-12.3f%-12.3f\n max        %-12.3f%-12.3f%-12.3f";
                break;
            case MILLISECONDS:
            default:
                this.resolution = MILLIS_IN_NANO;
                avgFormatStr = "%3.3fms";
                toStringFormatStr = hdr + "msecs\n%-12d%-12.3f%-12.3f%-12.3f\n min        %-12.3f%-12.3f%-12.3f\n max        %-12.3f%-12.3f%-12.3f";
                break;
        }
    }

    public void reset() {
        loopCount = 0;
        previousTime = System.nanoTime();
        movingTotal = 0;
        runningTotal = 0;
        movingAverage = 0;
        average = 0.0;
    }

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

    public long count() {
        return loopCount;
    }

    public double loopsSec() {
        return loopCount / (runningTotal / SECOND_IN_NANO);
    }

    public double movingAverage() {
        return movingAverage;
    }

    public double minMovingAverage() {
        return minMovingAverage;
    }

    public double maxMovingAverage() {
        return maxMovingAverage;
    }

    public String movingAverageString() {
        return String.format(avgFormatStr, movingAverage);
    }

    public double average() {
        return average;
    }

    public double minAverage() {
        return minAverage;
    }

    public double maxAverage() {
        return maxAverage;
    }

    public double minLoopTime() {
        return minLoopTime / resolution;
    }

    public double maxLoopTime() {
        return maxLoopTime / resolution;
    }

    public double elapsedTime() {
        return runningTotal / resolution;
    }

    public String averageString() {
        return String.format(avgFormatStr, average);
    }

    @Override
    public String toString() {
        return String.format(toStringFormatStr, loopCount, elapsedTime(), movingAverage, average, minLoopTime(), minMovingAverage, minAverage, maxLoopTime(), maxMovingAverage, maxAverage);
    }

    public enum Resolution {
        SECONDS,
        MILLISECONDS
    }
}
