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


}
