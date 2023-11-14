package org.firstinspires.ftc.teamcode.example.examplerobot.tasks;

import androidx.annotation.NonNull;
import org.firstinspires.ftc.teamcode.common.BunyipsOpMode;
import org.firstinspires.ftc.teamcode.common.tasks.Task;
import org.firstinspires.ftc.teamcode.example.examplerobot.components.ExampleDrive;

// Tasks are used to run OpMode code in Autonomous sections. It works by giving each task a specific
// amount of time to run, where it has it's own loop to run code in. This is useful for running
// specific tasks, such as moving the drive system for a certain amount of time, or until a certain
// condition is met.
public class ExampleTimeDriveTask extends Task {
    // You will need to store any COMPONENTS you wish to control in your task as a local instance
    private final ExampleDrive drive;

    // If you extend task without `time`, your task will never call itself finished!
    public ExampleTimeDriveTask(@NonNull BunyipsOpMode opMode, double time, ExampleDrive drive) {
        // Here, we allow the superclass to handle time and opMode. These are required.
        super(opMode, time);

        // Inside the constructor you can pass whatever you wish to control, for example, a drive
        this.drive = drive;
    }

    // Tasks have an init() method, which will run code once upon starting
    // the task. This is useful for resetting variables, or setting motors to run to position.
    @Override
    public void init() {
        // Init stuff here
    }

    // The run() method will run until the task timer runs out, or the finished state is set to true.
    @Override
    public void run() {
        drive.run(0.5);
        drive.update();
    }

    // Tasks have an optional isTaskFinished() method you can override, which will be the condition
    // that the task will stop running. This is useful for running a task until a certain condition
    // is met, such as a motor reaching a certain position.
    @Override
    public boolean isTaskFinished() {
        // Timeout will automatically be checked by the superclass, so you don't need to worry about
        // it. If you wish to check it yourself, your overhead caller should be checking isFinished(),
        // which internally checks isTaskFinished() and the timeout.
        // AutonomousBunyipsOpMode handles this automatically.
        return drive.isAtPosition(1000.0);
    }

    @Override
    public void onFinish() {
        // Runs as soon as the task is deemed 'finished'
        drive.run(0.0);
    }
}
