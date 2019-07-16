package com.qualcomm.robotcore.util;

/**
 * An approximation to the FTC ElapsedTime class
 */
public class ElapsedTime {

    private volatile long nsStartTime;

    public ElapsedTime(){
        nsStartTime = System.nanoTime();
    }

    /**
     * Reset the ElapsedTime to zero.
     */
    public void reset(){
        nsStartTime = System.nanoTime();
    }

    /**
     * Obtain milliseconds since the ElapsedTime object was created, or since the last call to reset().
     * @return elapsed time in ms
     */
    public double milliseconds(){
        return (double)(System.nanoTime() - nsStartTime) / 1000000.0;
    }

    /**
     * Obtain seconds since the ElapsedTime object was created, or since the last call to reset().
     * @return elapsed time in sec
     */
    public double seconds(){
        return (double)(System.nanoTime() - nsStartTime) / 1000000000.0;
    }

    /**
     * Obtain nanoseconds since the ElapsedTime object was created, or since the last call to reset().
     * @return elapsed time in nanoseconds.
     */
    public long nanoseconds(){
        return System.nanoTime() - nsStartTime;
    }
}
