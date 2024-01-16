package org.murraybridgebunyips.bunyipslib;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.murraybridgebunyips.bunyipslib.tasks.bases.Task;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.function.BooleanSupplier;

/**
 * Task scheduler for use with BunyipsLib.
 * @author Lucas Bubner, 2024
 */
public class Scheduler {
    private final OpMode opMode;
    private final ArrayList<BunyipsSubsystem> subsystems = new ArrayList<>();
    private final ArrayList<ConditionalTask> allocatedTasks = new ArrayList<>();

    public Scheduler(OpMode opMode) {
        this.opMode = opMode;
    }

    public void addSubsystems(BunyipsSubsystem... dispatch) {
        subsystems.addAll(Arrays.asList(dispatch));
    }

    public void run() {
        for (ConditionalTask task : allocatedTasks) {
            if (task.condition.getAsBoolean()) {
                task.activeSince = System.currentTimeMillis();
                if (
                    // While the time criterion is positive, the task will run for that many seconds
                    (task.timeCriterion > 0 && task.activeSince + task.timeCriterion * 1000 < System.currentTimeMillis()) ||
                    // While the time criterion is negative, the task will run in that many seconds
                    (task.timeCriterion < 0 && task.activeSince - task.timeCriterion * 1000 > System.currentTimeMillis()) ||
                    // While the time criterion is zero, the task will run immediately
                    (task.timeCriterion == 0)
                ) {
                    if (task.task.shouldOverrideOnConflict() == null) {
                        if (task.task.isFinished()) allocatedTasks.remove(task);
                        // This is a non-command task, run it now as it will not be run by any subsystem
                        task.task.run();
                        continue;
                    }
                    for (BunyipsSubsystem subsystem : subsystems) {
                        if (subsystem.getTaskDependencies().contains(task.task.hashCode())) {
                            subsystem.setCurrentTask(task.task);
                        }
                    }
                }
            } else {
                task.activeSince = -1;
            }
        }

        for (BunyipsSubsystem subsystem : subsystems) {
            subsystem.run();
        }
    }

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

    public ConditionalTask when(BooleanSupplier condition) {
        return new ConditionalTask(condition);
    }

    public ConditionalTask always() {
        return new ConditionalTask(
                () -> true
        );
    }

    private static class ControllerStateHandler implements BooleanSupplier {
        enum State {
            PRESSED,
            RELEASED,
            HELD
        }

        private OpMode opMode;
        private final State state;
        private final Controller.User user;
        private final Controller button;

        private boolean lockout = false;

        public ControllerStateHandler(OpMode opMode, Controller.User user, Controller button, State state) {
            this.user = user;
            this.button = button;
            this.state = state;
        }

        @Override
        public boolean getAsBoolean() {
            boolean isPressed = Controller.isSelected(user == Controller.User.ONE ? opMode.gamepad1 : opMode.gamepad2, button);
            switch (state) {
                case PRESSED:
                    if (isPressed && !lockout) {
                        lockout = true;
                        return true;
                    } else if (!isPressed) {
                        lockout = false;
                    }
                case RELEASED:
                    if (!isPressed && lockout) {
                        lockout = false;
                        return true;
                    } else if (isPressed) {
                        lockout = true;
                    }
                case HELD:
                    return isPressed;
                default:
                    return false;
            }
        }
    }

    public class ConditionalTask {
        protected Task task = null;
        protected double timeCriterion;
        protected BooleanSupplier condition;
        protected double activeSince;

        public ConditionalTask(BooleanSupplier condition) {
            this.condition = condition;
        }

        /**
         * Run a task when the condition is met.
         * This method can only be called once per ConditionalTask.
         * @param task The task to run.
         * @return Timing control for allocation (to queue this Task call forSeconds(), inSeconds(), or immediately()).
         */
        public ConditionalTask run(Task task) {
            if (this.task != null) {
                throw new EmergencyStop("A run() method has been called more than once on a scheduler task. If you wish to run multiple tasks see about using a task group as your task.");
            }
            this.task = task;
            return this;
        }

        public void forSeconds(double time) {
            timeCriterion = time;
            allocatedTasks.add(this);
        }

        public void inSeconds(double time) {
            timeCriterion = -time;
            allocatedTasks.add(this);
        }

        public void immediately() {
            timeCriterion = 0;
            allocatedTasks.add(this);
        }
    }
}
