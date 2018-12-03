package virtual_robot.controller;

import virtual_robot.controller.VirtualRobotController;

/**
 * OpModes in this simulator must extend LinearOpMode.
 */
public abstract class LinearOpMode extends VirtualRobotController.LinearOpModeBase {

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
    public LinearOpMode(){
        virtualRobotController.super();
    }

    /**
     * OpModes must override the abstract runOpMode() method.
     */
    public abstract void runOpMode();

    /**
     * Pause execution of the OpMode for the indicated number of milliseconds
     * @param milliseconds
     *
     * Note: during this pause, the motors will continue running at their previous power settings.
     */
    protected void sleep(long milliseconds){
        if (Thread.currentThread().isInterrupted()) return;
        try{
            Thread.sleep(milliseconds);
        } catch(InterruptedException exc){
            Thread.currentThread().interrupt();
        }
        return;
    }

    /**
     * Determines whether there has been a request to terminate execution of this OpMode (for example, clicking the "STOP"
     * button would result in such a request).
     *
     * This method also gives other threads an opportunity to run.
     *
     * Any long-running loop (more than a few iterations) should include a call to this method.
     *
     * @return TRUE if NO request to terminate; FALSE if there is a request to terminate.
     */
    protected boolean opModeIsActive(){
        if (Thread.currentThread().isInterrupted()) return false;
        try{
            Thread.sleep(0);
        } catch (InterruptedException exc){
            Thread.currentThread().interrupt();
            return false;
        }
        return true;
    }

}
