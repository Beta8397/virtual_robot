package org.murraybridgebunyips.bunyipslib.example.examplerobot.tasks;

import org.murraybridgebunyips.bunyipslib.Dbg;
import org.murraybridgebunyips.bunyipslib.example.examplerobot.components.ExampleLift;
import org.murraybridgebunyips.bunyipslib.tasks.bases.Task;

/**
 * Example task for a lift component.
 */
// Tasks are used to run OpMode code in Autonomous sections. It works by giving each task a specific
// amount of time to run, where it has it's own loop to run code in. This is useful for running
// specific tasks, such as moving the drive system for a certain amount of time, or until a certain
// condition is met. OpMode injection is now handled automatically with the use of a singleton property 'opMode',
// available in both a Task and a BunyipsComponent (and extensions).
// More recently, the Task system has expanded into TeleOp, where these Tasks represent commands when used with the Scheduler.
// See ExampleCommandBasedTeleOp.java for more information.
public class ExampleTask extends Task {
    // You will need to store any COMPONENTS you wish to control in your task as a local instance
    private final ExampleLift lift;

    /**
     * @param time the time to run the task for
     * @param lift the lift to control
     */
    public ExampleTask(double time, ExampleLift lift) {
        // Here, we allow the superclass to handle time controls. This is required.
        super(time);
        // Inside the constructor you can pass whatever you wish to control, for example, a drive
        this.lift = lift;
    }

    // Tasks have an init() method, which will run code once upon starting
    // the task. This is useful for resetting variables, or setting motors to run to position.
    @Override
    protected void init() {
        // Init stuff here
        lift.liftUp();
    }

    // The periodic() method will run until the task timer runs out, or the finished state is set to true.
    @Override
    protected void periodic() {
        lift.update();
    }

    // Tasks have an optional isTaskFinished() method you can override, which will be the condition
    // that the task will stop running. This is useful for running a task until a certain condition
    // is met, such as a motor reaching a certain position.
    @Override
    protected boolean isTaskFinished() {
        // Timeout will automatically be checked by the superclass, so you don't need to worry about
        // it. If you wish to check it yourself, your overhead caller should be checking isFinished(),
        // which internally checks isTaskFinished() and the timeout.
        // AutonomousBunyipsOpMode handles this automatically.
        return !lift.isBusy();
    }

    @Override
    protected void onFinish() {
        // Runs as soon as the task is deemed 'finished'
        Dbg.log("finished running exampletask");
    }
}
