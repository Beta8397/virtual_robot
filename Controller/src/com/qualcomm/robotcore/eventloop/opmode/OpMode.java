package com.qualcomm.robotcore.eventloop.opmode;

import virtual_robot.controller.VirtualRobotController;

import java.util.concurrent.TimeUnit;

/**
 * Virtual Robot's implementation of OpMode.
 *
 * Classes extending OpMode must define the "init" and "loop" methods.
 * Optionally, "init_loop", "start", and "stop" methods may be overridden.
 */
public abstract class OpMode extends VirtualRobotController.OpModeBase {
    /**
     * number of seconds this op mode has been running, this is
     * updated before every call to loop.
     */
    public double time = 0.0;

    // internal time tracking
    private long startTime; // in nanoseconds

    /**
     * Static reference to VirtualRobotController object, with getter and setter
     */
    private static VirtualRobotController virtualRobotController = null;

    public static VirtualRobotController getVirtualRobotController() {
        return virtualRobotController;
    }

    public static void setVirtualRobotController(VirtualRobotController controller) {
        virtualRobotController = controller;
    }

    /**
     * No-arg constructor -- requires that virtualRobotController be non-null
     */
    public OpMode() {
        virtualRobotController.super();
        startTime = System.nanoTime();
    }


    /**
     * User defined init method
     * <p>
     * This method will be called once when the INIT button is pressed.
     */
    abstract public void init();

    /**
     * User defined init_loop method
     * <p>
     * This method will be called repeatedly when the INIT button is pressed.
     * This method is optional. By default this method takes no action.
     */
    public void init_loop() {
    }


    /**
     * User defined start method.
     * <p>
     * This method will be called once when the PLAY button is first pressed.
     * This method is optional. By default this method takes not action.
     * Example usage: Starting another thread.
     */
    public void start() {
    }


    /**
     * User defined loop method
     * <p>
     * This method will be called repeatedly in a loop while this op mode is running
     */
    abstract public void loop();

    /**
     * User defined stop method
     * <p>
     * This method will be called when this op mode is first disabled
     * <p>
     * The stop method is optional. By default this method takes no action.
     */
    public void stop() {
    }

    /**
     * Do-nothing method
     */
    public final void requestOpModeStop() {
        stop();
    }

    /**
     * Get the number of seconds this op mode has been running
     * <p>
     * This method has sub millisecond accuracy.
     *
     * @return number of seconds this op mode has been running
     */
    public double getRuntime() {
        final double NANOSECONDS_PER_SECOND = TimeUnit.SECONDS.toNanos(1);
        return (System.nanoTime() - startTime) / NANOSECONDS_PER_SECOND;
    }

    /**
     * Reset the start time to zero.
     */
    public void resetStartTime() {
        startTime = System.nanoTime();
    }

    //----------------------------------------------------------------------------------------------
    // Safety Management
    //
    // These constants manage the duration we allow for callbacks to user code to run for before
    // such code is considered to be stuck (in an infinite loop, or wherever) and consequently
    // the robot controller application is restarted. They SHOULD NOT be modified except as absolutely
    // necessary as poorly chosen values might inadvertently compromise safety.
    //----------------------------------------------------------------------------------------------

    public int msStuckDetectInit = 5000;
    public int msStuckDetectInitLoop = 5000;
    public int msStuckDetectStart = 5000;
    public int msStuckDetectLoop = 5000;
    public int msStuckDetectStop = 1000;

    //----------------------------------------------------------------------------------------------
    // Internal
    //----------------------------------------------------------------------------------------------

    public void internalPreInit() {
    }

    /**
     * automatically update telemetry in a non-linear opmode
     */
    public void internalPostInitLoop() {
        telemetry.update();
    }

    /**
     * automatically update telemetry in a non-linear opmode
     */
    public void internalPostLoop() {
        telemetry.update();
    }
}
