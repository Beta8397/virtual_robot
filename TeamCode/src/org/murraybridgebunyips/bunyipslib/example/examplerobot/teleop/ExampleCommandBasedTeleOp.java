package org.murraybridgebunyips.bunyipslib.example.examplerobot.teleop;

import static org.murraybridgebunyips.bunyipslib.external.units.Units.Seconds;

import org.murraybridgebunyips.bunyipslib.BunyipsOpMode;
import org.murraybridgebunyips.bunyipslib.Controls;
import org.murraybridgebunyips.bunyipslib.Scheduler;
import org.murraybridgebunyips.bunyipslib.drive.TankDrive;
import org.murraybridgebunyips.bunyipslib.example.examplerobot.components.ExampleConfig;
import org.murraybridgebunyips.bunyipslib.tasks.ContinuousTask;
import org.murraybridgebunyips.bunyipslib.tasks.DifferentialDriveTask;
import org.murraybridgebunyips.bunyipslib.tasks.RunTask;

/**
 * Example command-based teleop
 *
 * @author Lucas Bubner, 2024
 */
// Alternatively CommandBasedBunyipsOpMode will give you direct access to the Scheduler, however this demo
// will not use it to prove how the command-based system works in unison with BunyipsOpMode.
public class ExampleCommandBasedTeleOp extends BunyipsOpMode {

    // Define subsystems and config
    private final ExampleConfig config = new ExampleConfig();
    // Command based OpModes hold onto a Scheduler, which is responsible for allocating tasks, commands, and subsystems.
    // The Scheduler is responsible for running the subsystems and commands, and is the only thing that should be
    // running in the activeLoop() method. Exceptions to this rule should be made wearily, and in CommandBasedBunyipsOpMode
    // this is by overriding the periodic() method which is not overridden by default.
    // Scheduler is also a component, so it has to be instantiated in the onInit() method.
    // CommandBasedBunyipsOpMode will handle this for you and all you will need to do is call
    // the scheduler() method to get the scheduler.
    private Scheduler scheduler;
    // All components and tasks must be instantiated during runtime, and not in the constructor or member fields.
    private TankDrive drive;

    @Override
    protected void onInit() {
        scheduler = new Scheduler();
        config.init();
        drive = new TankDrive(config.driveConstants, config.coefficients, config.imu, config.leftFrontMotor, config.leftBackMotor, config.leftBackMotor, config.rightBackMotor);

        // The scheduler must be initialised with all subsystems that will be used in the OpMode.
        // This is because the scheduler will be responsible for running the subsystems, and will
        // need to know what subsystems to call their update() methods on
        scheduler.addSubsystems(drive);
        // NOTE: CommandBasedBunyipsOpMode will handle this for you, by looking at all the constructed BunyipsSubsystems
        // and adding them automatically. If you don't want this behaviour, set your own subsystems that will be updated
        // through useSubsystems(), although you will receive warnings as a cleaner alternative is to call disable() on
        // all subsystems that don't want to be updated.

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
        // run callbacks on demand, such as RunTask, ConditionalTask, RunForTask, and others located in the
        // org.murraybridgebunyips.bunyipslib.tasks package.

        // Using the default DifferentialDriveTask from BunyipsLib, which will run the drive system based on gamepad1
        // left stick y and right stick x inputs.
        // All default tasks must never be able to finish, otherwise an EmergencyStop will be thrown.
        drive.setDefaultTask(new DifferentialDriveTask(gamepad1, drive));

        // Once your default tasks have been set, you can set other tasks that will run based on a boolean event,
        // or based on controller input. These tasks will replace the default tasks when the subsystem is mentioned as a task parameter,
        // or if no subsystem is attached it will run directly on the scheduler.
        scheduler.driver().whenPressed(Controls.A)
                .run(new RunTask(() -> telemetry.log("A was pressed!")));


        // There are plenty of configuration options, including setting a queue delay, stop condition for continuous tasks,
        // and more. See the Scheduler class and implementations for more information.
        scheduler.driver().whenHeld(Controls.B)
                .run(new RunTask(() -> telemetry.log("B started being held 3 seconds ago!")))
                .in(Seconds.of(3));

        scheduler.operator().whenReleased(Controls.X)
                // This will replace the default DifferentialDriveTask with this task, until X is pressed again
                .run(new ContinuousTask(() -> telemetry.add("X was released on gamepad2 and the drive system has been stopped.")).onSubsystem(drive, false))
                .finishingIf(() -> Controls.isSelected(gamepad2, Controls.X));

        scheduler.when(() -> drive.isBusy())
                .run(new RunTask(() -> telemetry.add("Drive system is busy!")));
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
