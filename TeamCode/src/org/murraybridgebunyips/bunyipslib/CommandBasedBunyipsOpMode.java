package org.murraybridgebunyips.bunyipslib;

/**
 * Command-based structure for a BunyipsOpMode utilising the Scheduler.
 * This can be used for seamless/zero-step integration with the Scheduler in TeleOp, for Autonomous it is
 * recommended to use the AutonomousBunyipsOpMode classes as Tasks there are used in a different context.
 *
 * @author Lucas Bubner, 2024
 */
public abstract class CommandBasedBunyipsOpMode extends BunyipsOpMode {
    // Components can't be final due to runtime instantiation,
    // so we cannot expose the scheduler directly and must use a getter.
    private Scheduler scheduler;

    /**
     * Call to access the Scheduler from within the OpMode.
     *
     * @return The Scheduler instance.
     */
    public Scheduler scheduler() {
        return scheduler;
    }

    @Override
    protected final void onInit() {
        scheduler = new Scheduler();
        onInitialisation();
        BunyipsSubsystem[] subsystems = setSubsystems();
        if (subsystems == null || subsystems.length == 0) {
            throw new EmergencyStop("No subsystems were set in setSubsystems() method");
        }
        scheduler.addSubsystems(subsystems);
        assignCommands();
    }

    @Override
    protected final void activeLoop() {
        periodic();
        scheduler.run();
    }

    // Access to the other BunyipsOpMode methods (onInitLoop() etc.) are handled by the user of this class, as
    // they may wish to do something else. This class does not limit the user from a standard BunyipsOpMode, but
    // removes the need to ensure the Scheduler is set up properly.

    /**
     * Runs upon the pressing of the INIT button on the Driver Station.
     * This is where you should initialise your hardware and other components.
     */
    protected abstract void onInitialisation();

    /**
     * Populate with the BunyipsSubsystems that need to be updated and managed by the Scheduler.
     *
     * @return {@code return new BunyipsSubsystem[] { ... };}
     */
    protected abstract BunyipsSubsystem[] setSubsystems();

    /**
     * Assign your scheduler commands here by accessing the `scheduler` field.
     */
    protected abstract void assignCommands();

    /**
     * Override this method to run any additional activeLoop() code before the Scheduler runs.
     */
    protected void periodic() {
    }
}
