package org.murraybridgebunyips.bunyipslib.example.examplerobot.teleop;

import org.murraybridgebunyips.bunyipslib.BunyipsOpMode;
import org.murraybridgebunyips.bunyipslib.Controller;
import org.murraybridgebunyips.bunyipslib.Scheduler;
import org.murraybridgebunyips.bunyipslib.drive.TankDrive;
import org.murraybridgebunyips.bunyipslib.example.examplerobot.components.ExampleConfig;
import org.murraybridgebunyips.bunyipslib.tasks.ContinuousTask;
import org.murraybridgebunyips.bunyipslib.tasks.DifferentialDriveTask;
import org.murraybridgebunyips.bunyipslib.tasks.InstantTask;

/**
 * Example command-based teleop
 *
 * @author Lucas Bubner, 2024
 */
// Alternatively CommandBasedBunyipsOpMode will give you direct access to the Scheduler, however this demo
// will not use it to prove how the command-based system works in unison with BunyipsOpMode.
public class ExampleCommandBasedTeleOp extends BunyipsOpMode {

    // Command based OpModes hold onto a Scheduler, which is responsible for allocating tasks, commands, and subsystems.
    // The Scheduler is responsible for running the subsystems and commands, and is the only thing that should be
    // running in the activeLoop() method. Exceptions to this rule should be made wearily, and in CommandBasedBunyipsOpMode
    // this is by overriding the periodic() method which is not overridden by default.
    private final Scheduler scheduler = new Scheduler(this);

    // Define subsystems and config
    private final ExampleConfig config = new ExampleConfig();
    private TankDrive drive;

    @Override
    protected void onInit() {
        config.init(this);
        drive = new TankDrive(this, config.driveConstants, config.coefficients, config.imu, config.leftFrontMotor, config.leftBackMotor, config.leftBackMotor, config.rightBackMotor);

        // The scheduler must be initialised with all subsystems that will be used in the OpMode.
        // This is because the scheduler will be responsible for running the subsystems, and will
        // need to know what subsystems to call their update() methods on
        scheduler.addSubsystems(drive);
        // CommandBasedBunyipsOpMode has a dedicated method to ensure your subsystems are added to the scheduler,
        // by requiring it be passed an array of subsystems in the setSubsystems() method

        // Here is where you will assign your commands. In CommandBasedBunyipsOpMode, you will have access to the
        // assignCommands() method, which internally is simply a function that is called after onInit(). Putting them
        // here will act the same way.

        // The scheduler exposes methods of grabbing gamepad input and BooleanSupplier events, which are used to
        // allocate tasks to subsystems or directly onto the scheduler. All tasks have a priority, which is used to
        // determine which task should be run first. The scheduler will replace tasks on subsystems that are set
        // to override, with the only exception being the default task, which will always run.

        // As such, the first priority is to set default tasks for your subsystems, which is a task that will always
        // run when the scheduler does not have any more tasks to run. This is useful for setting a default state for
        // your subsystems. This is the same as command-based programming in WPILib. Some tasks already exist that can
        // run callbacks on demand, such as InstantTask, ConditionalTask, RunForTask, and others located in the
        // org.murraybridgebunyips.bunyipslib.tasks package.

        // Using the default DifferentialDriveTask from BunyipsLib, which will run the drive system based on gamepad1
        // left stick y and right stick x inputs.
        // All default tasks must never be able to finish, otherwise an EmergencyStop will be thrown.
        drive.setDefaultTask(new DifferentialDriveTask(gamepad1, drive));

        // Once your default tasks have been set, you can set other tasks that will run based on a boolean event,
        // or based on controller input. These tasks will replace the default tasks when the subsystem is mentioned as a task parameter,
        // or if no subsystem is attached it will run directly on the scheduler.
        scheduler.whenPressed(Controller.User.ONE, Controller.A)
                .run(new InstantTask(() -> log("A was pressed!")))
                .immediately();

        // There are plenty of configuration options, including setting a queue delay, stop condition for continuous tasks,
        // and more. See the Scheduler class and implementations for more information.
        scheduler.whenHeld(Controller.User.ONE, Controller.B)
                .run(new InstantTask(() -> log("B started being held 3 seconds ago!")))
                .inSeconds(3);

        scheduler.whenReleased(Controller.User.TWO, Controller.X)
                // This will replace the default DifferentialDriveTask with this task, until X is pressed again
                .run(new ContinuousTask(() -> addTelemetry("X was released on gamepad2 and the drive system has been stopped."), drive, false))
                .finishingWhen(() -> Controller.isSelected(gamepad2, Controller.X));

        scheduler.when(() -> drive.isBusy())
                .run(new InstantTask(() -> addTelemetry("Drive system is busy!")))
                .immediately();
    }

    @Override
    protected void activeLoop() {
        // The only code that is required in the activeLoop is scheduler.run(), which will handle everything.
        // This method has been omitted from the CommandBasedBunyipsOpMode, as it is handled internally.
        // However, if absolutely necessary, you can override the periodic() method in CommandBasedBunyipsOpMode,
        // which is where you may add more activeLoop code. This is not recommended but remains an option.
        scheduler.run();
    }
}
