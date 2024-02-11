package org.murraybridgebunyips.bunyipslib;

import static org.murraybridgebunyips.bunyipslib.Text.formatString;
import static org.murraybridgebunyips.bunyipslib.Text.round;
import static org.murraybridgebunyips.bunyipslib.tasks.bases.Task.NANOS_IN_SECONDS;

import org.murraybridgebunyips.bunyipslib.tasks.bases.Task;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.function.BooleanSupplier;

/**
 * Scheduler and command plexus for use with the BunyipsLib task system.
 *
 * @author Lucas Bubner, 2024
 */
public class Scheduler extends BunyipsComponent {
    private static final ArrayList<String> subsystemReports = new ArrayList<>();
    private final ArrayList<BunyipsSubsystem> subsystems = new ArrayList<>();
    private final ArrayList<ConditionalTask> allocatedTasks = new ArrayList<>();

    /**
     * Used internally by subsystems to report their task-running status.
     */
    public static void addSubsystemTaskReport(String className, String taskName, double deltaTime) {
        subsystemReports.add(formatString("    % | % -> %s", className, taskName, deltaTime));
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
     * Run the scheduler. This will run all subsystems and tasks allocated to the scheduler.
     * This should be called in the activeLoop() method of the BunyipsOpMode.
     */
    public void run() {
        opMode.addTelemetry("Managing % task% (%sys, %cmd) on % subsystem%",
                // Subsystem count will account for default tasks
                allocatedTasks.size() + subsystems.size(),
                allocatedTasks.size() + subsystems.size() == 1 ? "" : "s",
                allocatedTasks.stream().filter(task -> task.task.shouldOverrideOnConflict() != null).count() + subsystems.size(),
                allocatedTasks.stream().filter(task -> task.task.shouldOverrideOnConflict() == null).count(),
                subsystems.size(),
                subsystems.size() == 1 ? "" : "s"
        );
        for (String item : subsystemReports) {
            opMode.addTelemetry(item);
        }
        for (ConditionalTask task : allocatedTasks) {
            if (task.task.shouldOverrideOnConflict() != null)
                continue;
            if (task.task.isRunning() || task.condition.getAsBoolean())
                opMode.addTelemetry("    Scheduler | % -> %s", task.task.getName(), round(task.task.getDeltaTime(), 1));
        }
        opMode.addTelemetry("");
        subsystemReports.clear();

        for (ConditionalTask task : allocatedTasks) {
            boolean condition = task.condition.getAsBoolean();
            if (condition || task.task.isRunning()) {
                // Latch current timing of truthy condition
                if (task.activeSince == -1) {
                    task.activeSince = System.nanoTime();
                }
                // Update controller states for determining whether they need to be continued to be run
                boolean timeoutExceeded = task.time * NANOS_IN_SECONDS + task.activeSince < System.nanoTime();
                if (task.condition instanceof ControllerStateHandler && task.time != 0.0) {
                    ((ControllerStateHandler) task.condition).setTimeoutCondition(timeoutExceeded);
                }
                // Trigger upon timeout goal or if the task does not have one
                if (task.time == 0.0 || timeoutExceeded) {
                    if (task.task.shouldOverrideOnConflict() == null) {
                        if (task.stopCondition.getAsBoolean()) {
                            // Finish handler will be called below
                            task.task.finishNow();
                        }
                        // Check for a debouncing situation and skip if it is not met
                        if (task.debouncing && !debouncingCheck(task, condition)) {
                            continue;
                        }
                        // This is a non-command task, run it now as it will not be run by any subsystem
                        task.task.run();
                        if (task.task.pollFinished()) {
                            // Reset the task as it is not attached to a subsystem and will not be reintegrated by one
                            task.task.reset();
                        }
                        continue;
                    }
                    // If it has a dependency, set the current task of the subsystems that depend on it
                    // Tasks may only have one subsystem dependency, where this dependency represents where the task
                    // will be executed by the scheduler.
                    for (BunyipsSubsystem subsystem : subsystems) {
                        if (task.stopCondition.getAsBoolean()) {
                            // Finish handler will be called on the subsystem
                            task.task.finishNow();
                        }
                        if (subsystem.getTaskDependencies().contains(task.task.hashCode())) {
                            subsystem.setCurrentTask(task.task);
                        }
                    }
                }
            } else {
                task.lastState = false;
                task.activeSince = -1;
            }
        }

        for (BunyipsSubsystem subsystem : subsystems) {
            subsystem.run();
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
     * Run a task when a controller button is held.
     *
     * @param user   The user of the controller.
     * @param button The button of the controller.
     * @return Timing/stop control for allocation.
     */
    public ConditionalTask whenHeld(Controller.User user, Controller button) {
        return new ConditionalTask(
                new ControllerStateHandler(
                        opMode,
                        user,
                        button,
                        ControllerStateHandler.State.HELD
                )
        );
    }

    /**
     * Run a task when a controller button is pressed (will run once when pressing the desired input).
     *
     * @param user   The user of the controller.
     * @param button The button of the controller.
     * @return Timing/stop control for allocation.
     */
    public ConditionalTask whenPressed(Controller.User user, Controller button) {
        return new ConditionalTask(
                new ControllerStateHandler(
                        opMode,
                        user,
                        button,
                        ControllerStateHandler.State.PRESSED
                )
        );
    }

    /**
     * Run a task when a controller button is released (will run once letting go of the desired input).
     *
     * @param user   The user of the controller.
     * @param button The button of the controller.
     * @return Timing/stop control for allocation.
     */
    public ConditionalTask whenReleased(Controller.User user, Controller button) {
        return new ConditionalTask(
                new ControllerStateHandler(
                        opMode,
                        user,
                        button,
                        ControllerStateHandler.State.RELEASED
                )
        );
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
        private final BunyipsOpMode opMode;
        private final State state;
        private final Controller.User user;
        private final Controller button;
        private final DebounceCondition debounceCondition;
        private boolean timerIsRunning;

        public ControllerStateHandler(BunyipsOpMode opMode, Controller.User user, Controller button, State state) {
            this.opMode = opMode;
            this.user = user;
            this.button = button;
            this.state = state;
            debounceCondition = new DebounceCondition(
                    () -> Controller.isSelected(Controller.getGamepad(user, opMode), button)
            );
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
                    return Controller.isSelected(Controller.getGamepad(user, opMode), button);
            }
            return false;
        }

        enum State {
            PRESSED,
            RELEASED,
            HELD
        }
    }

    public class ConditionalTask {
        protected final BooleanSupplier condition;
        protected Task task;
        protected double time;
        protected boolean debouncing;
        protected boolean lastState;
        protected BooleanSupplier stopCondition = () -> false;
        protected long activeSince = -1;

        public ConditionalTask(BooleanSupplier condition) {
            this.condition = condition;
        }

        /**
         * Run a task when the condition is met.
         * This method can only be called once per ConditionalTask.
         *
         * @param task The task to run.
         * @return Timing control for allocation (to queue this Task call forSeconds(), inSeconds(), or immediately()).
         */
        public ConditionalTask run(Task task) {
            if (this.task != null) {
                throw new EmergencyStop("A run(Task) method has been called more than once on a scheduler task. If you wish to run multiple tasks see about using a task group as your task.");
            }
            this.task = task;
            return this;
        }

        public ConditionalTask runDebounced(Task task) {
            debouncing = true;
            return run(task);
        }

        /**
         * Run a task assigned to in run() in a certain amount of seconds of the condition remaining true.
         * If on a controller, this will delay the activation of the task by the specified amount of seconds.
         *
         * @param time The amount of seconds to wait before running the task.
         */
        public void inSeconds(double time) {
            this.time = Math.abs(time);
            allocatedTasks.add(this);
        }

        /**
         * Run the task assigned to in run() until this condition is met. Once this condition is met, the task will
         * be forcefully stopped and the scheduler will move on. This is useful for continuous tasks.
         *
         * @param condition The condition to stop the task.
         */
        public void finishingWhen(BooleanSupplier condition) {
            stopCondition = condition;
            allocatedTasks.add(this);
        }

        /**
         * Run the task assigned to in run() in a certain amount of seconds of the condition remaining true.
         * If on a controller, this will delay the activation of the task by the specified amount of seconds.
         * Once this condition is met, the task will be forcefully stopped and the scheduler will move on.
         * This is useful for continuous tasks.
         *
         * @param time      The amount of seconds to wait before running the task.
         * @param condition The condition to stop the task.
         */
        public void inSecondsFinishingWhen(double time, BooleanSupplier condition) {
            this.time = Math.abs(time);
            stopCondition = condition;
            allocatedTasks.add(this);
        }

        /**
         * Run the task assigned to in run() immediately once the condition is true.
         */
        public void immediately() {
            time = 0.0;
            allocatedTasks.add(this);
        }
    }
}
