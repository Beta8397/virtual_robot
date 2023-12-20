package org.firstinspires.ftc.teamcode.common

/**
 * Time utilities for robot operation.
 * @author Shaun, 11/06/2017.
 */
class MovingAverageTimer @JvmOverloads constructor(
    bufferSize: Int = 100,
    resolution: Resolution? = Resolution.MILLISECONDS
) {
    // A ring buffer is used to keep track of a moving average
    private val ringBufferSize: Int
    private val loopTimeRingBuffer: LongArray
    private var resolution = 0.0
    private var avgFormatStr: String? = null
    private var toStringFormatStr: String? = null
    private var ringBufferIndex = 0
    private var loopCount: Long = 0
    private var movingTotal: Long = 0
    private var runningTotal: Long = 0
    private var previousTime: Long = 0
    private var movingAverage = 0.0
    private var minMovingAverage = Double.MAX_VALUE
    private var maxMovingAverage = Double.MIN_VALUE
    private var average = 0.0
    private var minAverage = Double.MAX_VALUE
    private var maxAverage = Double.MIN_VALUE
    private var minLoopTime = Double.MAX_VALUE
    private var maxLoopTime = Double.MIN_VALUE

    init {
        reset()
        ringBufferSize = bufferSize
        loopTimeRingBuffer = LongArray(ringBufferSize)
        val hdr = String.format("\n%-12s%-12s%-12s%-12s", "Loops", "TotalTime", "MovAvg", "Avg")
        when (resolution) {
            Resolution.SECONDS -> {
                this.resolution = SECOND_IN_NANO
                avgFormatStr = "%3.3f secs"
                toStringFormatStr =
                    "$hdr seconds\n%-12d%-12.3f%-12.3f%-12.3f\n min        %-12.3f%-12.3f%-12.3f\n max        %-12.3f%-12.3f%-12.3f"
            }

            Resolution.MILLISECONDS -> {
                this.resolution = MILLIS_IN_NANO
                avgFormatStr = "%3.3f ms"
                toStringFormatStr = """${hdr}ms
%-12d%-12.3f%-12.3f%-12.3f
 min        %-12.3f%-12.3f%-12.3f
 max        %-12.3f%-12.3f%-12.3f"""
            }

            else -> {
                this.resolution = MILLIS_IN_NANO
                avgFormatStr = "%3.3f ms"
                toStringFormatStr = """${hdr}ms
%-12d%-12.3f%-12.3f%-12.3f
 min        %-12.3f%-12.3f%-12.3f
 max        %-12.3f%-12.3f%-12.3f"""
            }
        }
    }

    fun reset() {
        loopCount = 0
        previousTime = System.nanoTime()
        movingTotal = 0
        runningTotal = 0
        movingAverage = 0.0
        average = 0.0
    }

    fun update() {
        val now = System.nanoTime()
        val loopTime = now - previousTime
        previousTime = now
        if (loopCount > 0) {
            minLoopTime = minLoopTime.coerceAtMost(loopTime.toDouble())
            maxLoopTime = maxLoopTime.coerceAtLeast(loopTime.toDouble())
        }

        // Adjust the running total
        movingTotal = movingTotal - loopTimeRingBuffer[ringBufferIndex] + loopTime
        runningTotal += loopTime

        // Add the new value
        loopTimeRingBuffer[ringBufferIndex] = loopTime

        // Wrap the current index
        ringBufferIndex = (ringBufferIndex + 1) % ringBufferSize
        loopCount += 1
        if (loopCount < ringBufferSize) {
            movingAverage = if (loopCount == 0L) {
                0.0
            } else {
                movingTotal.toDouble() / loopCount.toDouble() / resolution
            }
            // Temporarily fill the min/max movingAverage
            minMovingAverage = minMovingAverage.coerceAtMost(movingAverage)
            maxMovingAverage = maxMovingAverage.coerceAtLeast(movingAverage)
        } else {
            movingAverage = movingTotal.toDouble() / ringBufferSize.toDouble() / resolution

            // Reset the min/max movingAverage values the each time the buffer is filled
            if (ringBufferIndex == 0) {
                minMovingAverage = movingAverage
                maxMovingAverage = movingAverage
            } else {
                minMovingAverage = minMovingAverage.coerceAtMost(movingAverage)
                maxMovingAverage = maxMovingAverage.coerceAtLeast(movingAverage)
            }
        }
        average = runningTotal.toDouble() / loopCount / resolution
        minAverage = minAverage.coerceAtMost(average)
        maxAverage = maxAverage.coerceAtLeast(average)
    }

    fun count(): Long {
        return loopCount
    }

    fun movingAverage(): Double {
        return movingAverage
    }

    fun minMovingAverage(): Double {
        return minMovingAverage
    }

    fun maxMovingAverage(): Double {
        return maxMovingAverage
    }

    fun movingAverageString(): String {
        return String.format(avgFormatStr!!, movingAverage)
    }

    fun average(): Double {
        return average
    }

    fun minAverage(): Double {
        return minAverage
    }

    fun maxAverage(): Double {
        return maxAverage
    }

    fun minLoopTime(): Double {
        return minLoopTime / resolution
    }

    fun maxLoopTime(): Double {
        return maxLoopTime / resolution
    }

    fun elapsedTime(): Double {
        return runningTotal / resolution
    }

    fun averageString(): String {
        return String.format(avgFormatStr!!, average)
    }

    override fun toString(): String {
        return String.format(
            toStringFormatStr!!,
            loopCount,
            elapsedTime(),
            movingAverage,
            average,
            minLoopTime(),
            minMovingAverage,
            minAverage,
            maxLoopTime(),
            maxMovingAverage,
            maxAverage
        )
    }

    enum class Resolution {
        SECONDS, MILLISECONDS
    }

    companion object {
        const val SECOND_IN_NANO = 1000000000.0
        const val MILLIS_IN_NANO = 1000000.0
    }
}