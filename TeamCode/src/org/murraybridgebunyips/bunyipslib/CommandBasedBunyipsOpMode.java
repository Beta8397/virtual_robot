package org.murraybridgebunyips.bunyipslib;

import androidx.annotation.NonNull;

import java.util.Arrays;
import java.util.Collections;
import java.util.HashSet;
import java.util.Optional;
import java.util.function.BooleanSupplier;

/**
 * Command-based structure for a {@link BunyipsOpMode} utilising the Scheduler.
 * This can be used for seamless/zero-step integration with the Scheduler in TeleOp, for Autonomous it is
 * recommended to use the {@link AutonomousBunyipsOpMode} classes as Tasks there are used in a different context.
 *
 * @author Lucas Bubner, 2024
 * @see BunyipsOpMode
 */
public abstract class CommandBasedBunyipsOpMode extends BunyipsOpMode {
    @NonNull
    private HashSet<BunyipsSubsystem> managedSubsystems = new HashSet<>();

    // Components can't be final due to runtime instantiation, so we cannot expose the scheduler directly and must use a getter.
    // This is for consistency with the driver() and operator() methods, which are also accessed through a getter.
    // We technically could assign this similar to the telemetry, gamepads, and timer in BunyipsOpMode, but this is more explicit
    // and reduces the risk of accidental overwriting of this field especially as this is a derived class.
    private Scheduler scheduler;

    /**
     * Call to directly access the Scheduler from within the OpMode.
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
     * Call to access the when() scheduler from the Scheduler.
     * This is the same as calling scheduler().when().
     * This is used to create a conditional command based on a boolean supplier.
     *
     * @param condition the condition to be checked
     * @return task creation
     */
    public Scheduler.ConditionalTask when(BooleanSupplier condition) {
        return scheduler.when(condition);
    }

    /**
     * Call to access the whenDebounced() scheduler from the Scheduler.
     * This is the same as calling scheduler().whenDebounced().
     * This is used to create a conditional command based on a boolean supplier that is debounced.
     *
     * @param condition the condition to be debounced
     * @return task creation
     */
    public Scheduler.ConditionalTask whenDebounced(BooleanSupplier condition) {
        return scheduler.whenDebounced(condition);
    }

    /**
     * Call to access the always() scheduler from the Scheduler.
     * This is the same as calling scheduler().always().
     * This is used to create a command that runs every loop.
     *
     * @return task creation
     */
    public Scheduler.ConditionalTask always() {
        return scheduler.always();
    }

    /**
     * Call to manually add the subsystems that should be managed by the scheduler. Using this method will override
     * the automatic collection of {@link BunyipsSubsystem}s, and allows you to determine which subsystems will be managed for
     * this OpMode.
     * <p>
     * For most cases, using this method is not required and all you need to do is construct your subsystems and they
     * will be managed automatically. This method is for advanced cases where you don't want this behaviour to happen.
     *
     * @param subsystems the restrictive list of subsystems to be managed and updated by the scheduler
     */
    public void useSubsystems(BunyipsSubsystem... subsystems) {
        if (!NullSafety.assertNotNull(Arrays.stream(subsystems).toArray())) {
            throw new RuntimeException("Null subsystems were added in the useSubsystems() method!");
        }
        Collections.addAll(managedSubsystems, subsystems);
    }

    @Override
    protected final void onInit() {
        try {
            onInitialise();
        } catch (Exception e) {
            Exceptions.handle(e, telemetry::log);
        }
        scheduler = new Scheduler();
        if (managedSubsystems.isEmpty()) {
            managedSubsystems = BunyipsSubsystem.getInstances();
        }
        scheduler.addSubsystems(managedSubsystems.toArray(new BunyipsSubsystem[0]));
        assignCommands();
        Scheduler.ConditionalTask[] tasks = scheduler.getAllocatedTasks();
        BunyipsSubsystem[] subsystems = scheduler.getManagedSubsystems();
        Text.Builder out = Text.builder();
        // Task count will account for tasks on subsystems that are not IdleTasks
        int taskCount = (int) (tasks.length + subsystems.length - Arrays.stream(subsystems).filter(BunyipsSubsystem::isIdle).count());
        out.append("[CommandBasedBunyipsOpMode] assignCommands() called | Managing % subsystem(s) | % task(s) scheduled (% subsystem, % command)\n",
                subsystems.length,
                taskCount,
                Arrays.stream(tasks).filter(task -> task.taskToRun.hasDependency()).count() + taskCount - tasks.length,
                Arrays.stream(tasks).filter(task -> !task.taskToRun.hasDependency()).count()
        );
        for (BunyipsSubsystem subsystem : subsystems) {
            out.append("  | %\n", subsystem.toVerboseString());
            for (Scheduler.ConditionalTask task : tasks) {
                Optional<BunyipsSubsystem> dep = task.taskToRun.getDependency();
                if (!dep.isPresent() || !dep.get().equals(subsystem)) continue;
                out.append("    -> %\n", task);
            }
        }
        for (Scheduler.ConditionalTask task : tasks) {
            if (task.taskToRun.getDependency().isPresent()) continue;
            out.append("  | %\n", task);
        }
        Dbg.logd(out.toString());
        // Ensure to always run assignCommands() even if no subsystems are made, since it may be used for other purposes
        if (managedSubsystems.isEmpty()) {
            throw new RuntimeException("No BunyipsSubsystems were constructed!");
        }
    }

    @Override
    protected final void activeLoop() {
        try {
            periodic();
        } catch (Exception e) {
            Exceptions.handle(e, telemetry::log);
        }
        scheduler.run();
    }

    // Access to the other BunyipsOpMode methods (onInitLoop() etc.) are handled by the user of this class, as
    // they may wish to do something else. This class does not limit the user from a standard BunyipsOpMode, but
    // removes the need to ensure the Scheduler is set up properly.

    /**
     * Runs upon the pressing of the INIT button on the Driver Station.
     * This is where you should initialise your hardware and other components.
     */
    protected abstract void onInitialise();

    /**
     * Assign your scheduler commands here by accessing the {@link #scheduler()} and controllers {@link #driver()} and {@link #operator()}.
     */
    protected abstract void assignCommands();

    /**
     * Override this method to run any additional activeLoop() code before the Scheduler runs.
     */
    protected void periodic() {
    }
}
