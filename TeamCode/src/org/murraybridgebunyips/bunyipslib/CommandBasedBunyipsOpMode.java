package org.murraybridgebunyips.bunyipslib;

import java.util.Arrays;
import java.util.Collections;
import java.util.HashSet;

/**
 * Command-based structure for a {@link BunyipsOpMode} utilising the Scheduler.
 * This can be used for seamless/zero-step integration with the Scheduler in TeleOp, for Autonomous it is
 * recommended to use the {@link AutonomousBunyipsOpMode} classes as Tasks there are used in a different context.
 *
 * @author Lucas Bubner, 2024
 * @see BunyipsOpMode
 */
public abstract class CommandBasedBunyipsOpMode extends BunyipsOpMode {
    private final HashSet<BunyipsSubsystem> managedSubsystems = new HashSet<>();

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

    /**
     * Call to access the driver() method from the Scheduler.
     * This is the same as calling scheduler().driver().
     *
     * @return a ControllerButtonCreator for the driver controller
     */
    public Scheduler.ControllerButtonCreator driver() {
        return scheduler.driver();
    }

    /**
     * Call to access the operator() method from the Scheduler.
     * This is the same as calling scheduler().operator().
     *
     * @return a ControllerButtonCreator for the operator controller
     */
    public Scheduler.ControllerButtonCreator operator() {
        return scheduler.operator();
    }

    /**
     * Call to add subsystems that should be managed by the Scheduler. This is required to be
     * called in the onInitialisation() method, otherwise your subsystems will not be updated.
     *
     * @param subsystems the subsystems to be managed and updated by the Scheduler
     */
    public void addSubsystems(BunyipsSubsystem... subsystems) {
        if (!NullSafety.assertNotNull(Arrays.stream(subsystems).toArray())) {
            throw new RuntimeException("Null subsystems were added in the addSubsystems() method!");
        }
        Collections.addAll(managedSubsystems, subsystems);
    }

    @Override
    protected final void onInit() {
        onInitialisation();
        scheduler = new Scheduler();
        if (managedSubsystems == null || managedSubsystems.isEmpty()) {
            throw new RuntimeException("No BunyipsSubsystems were added in the addSubsystems() method!");
        }
        scheduler.addSubsystems(managedSubsystems.toArray(new BunyipsSubsystem[0]));
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
     * Assign your scheduler commands here by accessing the {@link #scheduler()}.
     */
    protected abstract void assignCommands();

    /**
     * Override this method to run any additional activeLoop() code before the Scheduler runs.
     */
    protected void periodic() {
    }
}
