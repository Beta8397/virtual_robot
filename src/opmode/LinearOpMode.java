package opmode;

import controller.VirtualRobotApplication;
import controller.VirtualRobotController;
import controller.VirtualRobotController.LinearOpModeBase;

public abstract class LinearOpMode extends VirtualRobotController.LinearOpModeBase {

    public abstract void runOpMode();

    protected void sleep(long milliseconds){
        if (Thread.currentThread().isInterrupted()) return;
        try{
            Thread.sleep(milliseconds);
        } catch(InterruptedException exc){
            Thread.currentThread().interrupt();
        }
        return;
    }

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
