package org.murraybridgebunyips.bunyipslib;

import static org.murraybridgebunyips.bunyipslib.Text.formatString;
import static org.murraybridgebunyips.bunyipslib.Text.round;
import static org.murraybridgebunyips.bunyipslib.external.units.Units.Nanoseconds;
import static org.murraybridgebunyips.bunyipslib.external.units.Units.Seconds;
import static org.murraybridgebunyips.bunyipslib.tasks.bases.Task.INFINITE_TIMEOUT;

import org.murraybridgebunyips.bunyipslib.external.units.Measure;
import org.murraybridgebunyips.bunyipslib.external.units.Time;
import org.murraybridgebunyips.bunyipslib.tasks.RunTask;
import org.murraybridgebunyips.bunyipslib.tasks.bases.Task;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.function.BooleanSupplier;
import java.util.function.Predicate;

/**
 * Scheduler and command plexus for use with the BunyipsLib task system.
 *
 * @author Lucas Bubner, 2024
 * @see CommandBasedBunyipsOpMode
 */
public class Scheduler extends BunyipsComponent {
    private static final ArrayList<String> reports = new ArrayList<>();
    private static boolean isMuted = false;
    private final ArrayList<BunyipsSubsystem> subsystems = new ArrayList<>();
    private final ArrayList<ConditionalTask> allocatedTasks = new ArrayList<>();

    /**
     * Create a new scheduler and reset static fields.
     */
    public Scheduler() {
        isMuted = false;
        reports.clear();
    }

    /**
     * Used internally by subsystems and tasks to report their running status statically.
     * This method is not intended for use by the user.
     *
     * @param className     The class name of the subsystem or context.
     * @param isDefaultTask Whether this task is a default task.
     * @param taskName      The name of the task.
     * @param deltaTimeSec  The time this task has been running in seconds
     * @param timeoutSec    The time this task is allowed to run in seconds, 0.0 if indefinite
     */
    public static void addTaskReport(String className, boolean isDefaultTask, String taskName, double deltaTimeSec, double timeoutSec) {
        if (isMuted) return;
        String report = formatString(
                "<small><b>%</b>% <font color='gray'>|</font> <b>%</b> -> %",
                className,
                isDefaultTask ? " (d.)" : "",
                taskName,
                deltaTimeSec
        );
        report += timeoutSec == 0.0 ? "s" : "/" + timeoutSec + "s";
        reports.add(report + "</small>");
    }

    /**
     * Add subsystems to the scheduler. This will ensure the update() method of the subsystems is called.
     *
     * @param dispatch The subsystems to add.
     */
    public void addSubsystems(BunyipsSubsystem... dispatch) {
        subsystems.addAll(Arrays.asList(dispatch));
    }

    /**
     * Disable all subsystems attached to the Scheduler.
     */
    public void disable() {
        for (BunyipsSubsystem subsystem : subsystems) {
            subsystem.disable();
        }
    }

    /**
     * Enable all subsystems attached to the Scheduler, unless they failed from null assertion.
     */
    public void enable() {
        for (BunyipsSubsystem subsystem : subsystems) {
            subsystem.enable();
        }
    }

    /**
     * Mute Scheduler telemetry.
     */
    public void mute() {
        isMuted = true;
    }

    /**
     * Unmute Scheduler telemetry.
     */
    public void unmute() {
        isMuted = false;
    }

    private boolean timeExceeded(ConditionalTask task) {
        return task.time.in(Nanoseconds) + task.activeSince < System.nanoTime();
    }

    /**
     * Run the scheduler. This will run all subsystems and tasks allocated to the scheduler.
     * This should be called in the {@code activeLoop()} method of the {@link BunyipsOpMode}, and is automatically called
     * in {@link CommandBasedBunyipsOpMode}.
     */
    public void run() {
        if (!isMuted) {
            // Task count will account for tasks on subsystems that are not IdleTasks
            int taskCount = (int) (allocatedTasks.size() + subsystems.size() - subsystems.stream().filter(BunyipsSubsystem::isIdle).count());
            opMode.addTelemetry("Managing % task% (%s, %c) on % subsystem%",
                    taskCount,
                    taskCount == 1 ? "" : "s",
                    allocatedTasks.stream().filter(task -> task.taskToRun.hasDependency()).count() + taskCount - allocatedTasks.size(),
                    allocatedTasks.stream().filter(task -> !task.taskToRun.hasDependency()).count(),
                    subsystems.size(),
                    subsystems.size() == 1 ? "" : "s"
            );
            for (String item : reports) {
                if (item.contains("IdleTask")) continue;
                opMode.addTelemetry(item);
            }
            for (ConditionalTask task : allocatedTasks) {
                if (task.taskToRun.hasDependency() // Whether the task is never run from the Scheduler (and task reports will come from the reports array)
                        || task.debouncing // Whether this task will only run once and then proceed to stay as "active" in the telemetry
                        || task.taskToRun.isMuted() // Whether the task has declared itself as muted
                        || task.activeSince == -1 // Whether the task is not being run by the Scheduler currently
                        || !timeExceeded(task)) { // Whether this task has not met timeout requirements
                    continue;
                }
                double deltaTime = round(task.taskToRun.getDeltaTime().in(Seconds), 1);
                opMode.addTelemetry(
                        "<small><b>Scheduler</b> (c.) <font color='gray'>|</font> <b>%</b> -> %</small>",
                        task.taskToRun,
                        deltaTime == 0.0 ? "active" : deltaTime + "s"
                );
            }
            // Blank line to separate Scheduler information from addTelemetry()
            opMode.addTelemetry("");
            reports.clear();
        }

        for (ConditionalTask task : allocatedTasks) {
            boolean condition = task.runCondition.getAsBoolean();
            if (condition || task.taskToRun.isRunning()) {
                // Latch current timing of truthy condition
                if (task.activeSince == -1) {
                    task.activeSince = System.nanoTime();
                }
                // Update controller states for determining whether they need to be continued to be run
                boolean timeoutExceeded = timeExceeded(task);
                if (task.runCondition instanceof ControllerStateHandler && !task.time.equals(INFINITE_TIMEOUT)) {
                    ((ControllerStateHandler) task.runCondition).setTimeoutCondition(timeoutExceeded);
                }
                // Trigger upon timeout goal or if the task does not have one
                if (task.time.equals(INFINITE_TIMEOUT) || timeoutExceeded) {
                    if (!task.taskToRun.hasDependency()) {
                        if (task.stopCondition.getAsBoolean()) {
                            // Finish handler will be called below
                            task.taskToRun.finish();
                        }
                        // Check for a debouncing situation and skip if it is not met
                        if (task.debouncing && !debouncingCheck(task, condition)) {
                            continue;
                        }
                        // This is a non-command task, run it now as it will not be run by any subsystem
                        task.taskToRun.run();
                        if (task.taskToRun.pollFinished()) {
                            // Reset the task as it is not attached to a subsystem and will not be reintegrated by one
                            task.taskToRun.reset();
                        }
                        continue;
                    }
                    // This task must have a dependency, set the current task of the subsystem that depends on it
                    // Tasks may only have one subsystem dependency, where this dependency represents where the task
                    // will be executed by the scheduler.
                    assert task.taskToRun.getDependency().isPresent();
                    if (task.stopCondition.getAsBoolean()) {
                        // Finish handler will be called on the subsystem
                        task.taskToRun.finish();
                    }
                    task.taskToRun.getDependency().get().setCurrentTask(task.taskToRun);
                }
            } else {
                task.lastState = false;
                task.activeSince = -1;
            }
        }

        for (BunyipsSubsystem subsystem : subsystems) {
            subsystem.update();
        }
    }

    private boolean debouncingCheck(ConditionalTask task, boolean condition) {
        if (condition && !task.lastState) {
            task.lastState = true;
            return true;
        } else if (!condition) {
            // This reset is also handled if the condition is false by the main loop
            task.lastState = false;
        }
        return false;
    }

    /**
     * Create a new controller button trigger creator.
     *
     * @param user The driver to use for the controller.
     * @return The controller button trigger creator.
     */
    public ControllerButtonCreator when(Controller user) {
        return new ControllerButtonCreator(user);
    }

    /**
     * Create a new controller button trigger creator for the driver.
     *
     * @return The controller button trigger creator.
     */
    public ControllerButtonCreator driver() {
        return new ControllerButtonCreator(opMode.gamepad1);
    }

    /**
     * Create a new controller button trigger creator for the operator.
     *
     * @return The controller button trigger creator.
     */
    public ControllerButtonCreator operator() {
        return new ControllerButtonCreator(opMode.gamepad2);
    }

    /**
     * Run a task when a condition is met.
     *
     * @param condition Supplier to provide a boolean value of when the task should be run.
     * @return Timing/stop control for allocation.
     */
    public ConditionalTask when(BooleanSupplier condition) {
        return new ConditionalTask(condition);
    }

    /**
     * Run a task when a condition is met, debouncing the task from running more than once the condition is met.
     *
     * @param condition Supplier to provide a boolean value of when the task should be run.
     * @return Timing/stop control for allocation.
     * @see DebounceCondition
     */
    public ConditionalTask whenDebounced(BooleanSupplier condition) {
        return new ConditionalTask(
                new DebounceCondition(condition)
        );
    }

    /**
     * Run a task always. This is the same as calling .when(() -> true).
     *
     * @return Timing/stop control for allocation.
     */
    public ConditionalTask always() {
        return new ConditionalTask(
                () -> true
        );
    }

    private static class DebounceCondition implements BooleanSupplier {
        private final BooleanSupplier condition;
        private boolean lastState;

        public DebounceCondition(BooleanSupplier condition) {
            this.condition = condition;
        }

        @Override
        public boolean getAsBoolean() {
            boolean currentState = condition.getAsBoolean();
            if (currentState && !lastState) {
                lastState = true;
                return true;
            } else if (!currentState) {
                lastState = false;
            }
            return false;
        }

        public boolean getReversedAsBoolean() {
            boolean currentState = condition.getAsBoolean();
            if (!currentState && lastState) {
                lastState = false;
                return true;
            } else if (currentState) {
                lastState = true;
            }
            return false;
        }
    }

    private static class ControllerStateHandler implements BooleanSupplier {
        private final State state;
        private final Controls button;
        private final Controller controller;
        private final DebounceCondition debounceCondition;
        private boolean timerIsRunning;

        public ControllerStateHandler(Controller controller, Controls button, State state) {
            this.button = button;
            this.state = state;
            this.controller = controller;
            debounceCondition = new DebounceCondition(() -> controller.get(button));
        }

        public void setTimeoutCondition(boolean timerNotFinished) {
            timerIsRunning = !timerNotFinished;
        }

        @Override
        public boolean getAsBoolean() {
            switch (state) {
                case PRESSED:
                    // Allow timers to run if they exist by not locking out but returning true for the duration
                    // This also ensures that repeated calls will not trigger the task multiple times, as this adds
                    // way too much unnecessary complexity to the task allocation system and to the BunyipsOpMode itself.
                    // There is realistically no reason for a task to have such a delay between allocation and execution
                    // where it will be called multiple times, in which case the task itself should be the one waiting.
                    return debounceCondition.getAsBoolean() || timerIsRunning;
                case RELEASED:
                    return debounceCondition.getReversedAsBoolean() || timerIsRunning;
                case HELD:
                    return controller.get(button);
            }
            return false;
        }

        enum State {
            PRESSED,
            RELEASED,
            HELD
        }
    }

    /**
     * Controller button trigger creator.
     * Used for ControllerStateHandler creation.
     */
    public class ControllerButtonCreator {
        private final Controller user;

        private ControllerButtonCreator(Controller user) {
            this.user = user;
        }

        /**
         * Run a task once this analog axis condition is met.
         *
         * @param axis      The axis of the controller.
         * @param threshold The threshold to meet.
         * @return Timing/stop control for allocation.
         */
        public ConditionalTask when(Controls.Analog axis, Predicate<? super Float> threshold) {
            return new ConditionalTask(
                    () -> threshold.test(user.get(axis))
            );
        }

        /**
         * Run a task when a controller button is held.
         *
         * @param button The button of the controller.
         * @return Timing/stop control for allocation.
         */
        public ConditionalTask whenHeld(Controls button) {
            return new ConditionalTask(
                    new ControllerStateHandler(
                            user,
                            button,
                            ControllerStateHandler.State.HELD
                    )
            );
        }

        /**
         * Run a task when a controller button is pressed (will run once when pressing the desired input).
         *
         * @param button The button of the controller.
         * @return Timing/stop control for allocation.
         */
        public ConditionalTask whenPressed(Controls button) {
            return new ConditionalTask(
                    new ControllerStateHandler(
                            user,
                            button,
                            ControllerStateHandler.State.PRESSED
                    )
            );
        }

        /**
         * Run a task when a controller button is released (will run once letting go of the desired input).
         *
         * @param button The button of the controller.
         * @return Timing/stop control for allocation.
         */
        public ConditionalTask whenReleased(Controls button) {
            return new ConditionalTask(
                    new ControllerStateHandler(
                            user,
                            button,
                            ControllerStateHandler.State.RELEASED
                    )
            );
        }
    }

    /**
     * A task that will run when a condition is met.
     */
    public class ConditionalTask {
        protected final BooleanSupplier runCondition;
        private final ArrayList<BooleanSupplier> and = new ArrayList<>();
        private final ArrayList<BooleanSupplier> or = new ArrayList<>();
        protected Task taskToRun;
        protected Measure<Time> time = INFINITE_TIMEOUT;
        protected boolean debouncing;
        protected boolean lastState;
        protected BooleanSupplier stopCondition = () -> false;
        protected long activeSince = -1;
        private boolean isTaskMuted = false;

        /**
         * Create and allocate a new conditional task. This will automatically be added to the scheduler.
         *
         * @param runCondition The condition to start running the task.
         */
        public ConditionalTask(BooleanSupplier runCondition) {
            // Run the task if the original expression is met,
            // and all AND conditions are met, or any OR conditions are met
            this.runCondition = () -> runCondition.getAsBoolean()
                    && and.stream().allMatch(BooleanSupplier::getAsBoolean)
                    || or.stream().anyMatch(BooleanSupplier::getAsBoolean);
            allocatedTasks.add(this);
        }

        /**
         * Run a task when the condition is met.
         * This method can only be called once per ConditionalTask.
         * If you do not mention timing control, this task will be run immediately when the condition is met,
         * ending when the task ends.
         *
         * @param task The task to run.
         * @return Current builder for additional task parameters
         */
        public ConditionalTask run(Task task) {
            if (taskToRun != null) {
                throw new EmergencyStop("A run(Task) method has been called more than once on a scheduler task. If you wish to run multiple tasks see about using a task group as your task.");
            }
            taskToRun = task;
            if (isTaskMuted)
                taskToRun.withMutedReports();
            return this;
        }

        /**
         * Implicitly make a new RunTask to run once the condition is met.
         * This method can only be called once per ConditionalTask.
         * If you do not mention timing control, this task will be run immediately when the condition is met,
         * ending immediately as it is an RunTask.
         *
         * @param runnable The code to run
         * @return Current builder for additional task parameters
         */
        public ConditionalTask run(Runnable runnable) {
            return run(new RunTask(runnable));
        }

        /**
         * Run a task when the condition is met, debouncing the task from running more than once the condition is met.
         * This method can only be called once per ConditionalTask.
         * If you do not mention timing control, this task will be run immediately when the condition is met,
         * ending when the task ends.
         *
         * @param task The task to run.
         * @return Current builder for additional task parameters
         */
        public ConditionalTask runDebounced(Task task) {
            debouncing = true;
            return run(task);
        }

        /**
         * Implicitly make a new RunTask to run once the condition is met, debouncing the task from running more than once the condition is met.
         * This method can only be called once per ConditionalTask.
         * If you do not mention timing control, this task will be run immediately when the condition is met,
         * ending immediately as it is an RunTask.
         *
         * @param runnable The code to run
         * @return Current builder for additional task parameters
         */
        public ConditionalTask runDebounced(Runnable runnable) {
            return runDebounced(new RunTask(runnable));
        }

        /**
         * Mute this task from being a part of the Scheduler report.
         *
         * @return Current builder for additional task parameters
         */
        public ConditionalTask muted() {
            if (taskToRun != null) {
                taskToRun.withMutedReports();
            }
            isTaskMuted = true;
            return this;
        }

        /**
         * Chain an AND condition to the current conditional task.
         * Will be evaluated after the controller condition, and before the OR conditions.
         *
         * @param condition The AND condition to chain.
         * @return Current builder for additional task parameters
         */
        public ConditionalTask andIf(BooleanSupplier condition) {
            and.add(condition);
            return this;
        }

        /**
         * Chain an OR condition to the current conditional task.
         * Will be evaluated after the controller and AND conditions.
         *
         * @param condition The OR condition to chain.
         * @return Current builder for additional task parameters
         */
        public ConditionalTask orIf(BooleanSupplier condition) {
            or.add(condition);
            return this;
        }

        /**
         * Run a task assigned to in run() in a certain amount of time of the condition remaining true.
         * If on a controller, this will delay the activation of the task by the specified amount of time.
         *
         * @param interval The time interval
         */
        public void in(Measure<Time> interval) {
            time = interval;
        }

        /**
         * Run a task assigned to in run() in a certain amount of time of the condition remaining true.
         * If on a controller, this will delay the activation of the task by the specified amount of time.
         *
         * @param interval The time interval
         */
        // Kotlin interop, as in is a reserved keyword
        public void inTime(Measure<Time> interval) {
            time = interval;
        }

        /**
         * Run the task assigned to in run() until this condition is met. Once this condition is met, the task will
         * be forcefully stopped and the scheduler will move on. This is useful for continuous tasks.
         *
         * @param condition The condition to stop the task. Note the task will be auto-stopped if it finishes by itself,
         *                  this condition simply allows for an early finish if this condition is met.
         */
        public void finishingIf(BooleanSupplier condition) {
            stopCondition = condition;
        }

        /**
         * Run the task assigned to in run() in a certain amount of time of the condition remaining true.
         * If on a controller, this will delay the activation of the task by the specified amount of time.
         * Once this condition is met, the task will be forcefully stopped and the scheduler will move on.
         * This is useful for continuous tasks.
         *
         * @param interval  The time interval
         * @param condition The condition to stop the task. Note the task will be auto-stopped if it finishes by itself,
         *                  this condition simply allows for an early finish if this condition is met.
         */
        public void inTimeFinishingIf(Measure<Time> interval, BooleanSupplier condition) {
            time = interval;
            stopCondition = condition;
        }
    }
}
