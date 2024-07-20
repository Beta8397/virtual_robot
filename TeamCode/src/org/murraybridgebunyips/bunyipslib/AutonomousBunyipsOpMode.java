package org.murraybridgebunyips.bunyipslib;

import static org.murraybridgebunyips.bunyipslib.Text.formatString;
import static org.murraybridgebunyips.bunyipslib.Text.round;
import static org.murraybridgebunyips.bunyipslib.external.units.Units.Seconds;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import org.jetbrains.annotations.NotNull;
import org.murraybridgebunyips.bunyipslib.external.units.Measure;
import org.murraybridgebunyips.bunyipslib.external.units.Time;
import org.murraybridgebunyips.bunyipslib.tasks.RunTask;
import org.murraybridgebunyips.bunyipslib.tasks.bases.RobotTask;
import org.murraybridgebunyips.bunyipslib.tasks.bases.Task;
import org.murraybridgebunyips.bunyipslib.tasks.groups.TaskGroup;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.HashSet;
import java.util.Iterator;
import java.util.List;
import java.util.concurrent.ConcurrentLinkedDeque;
import java.util.concurrent.atomic.AtomicBoolean;

/**
 * {@link BunyipsOpMode} variant for Autonomous operation. Uses the {@link Task} system for a queued action OpMode.
 *
 * @author Lucas Bubner, 2023
 * @author Lachlan Paul, 2023
 * @see RoadRunner
 * @see BunyipsOpMode
 */
public abstract class AutonomousBunyipsOpMode extends BunyipsOpMode {
    // Used for tasks that have no timeout to generate a "estimate to OpMode completion" metric
    private static final double INFINITE_TASK_ASSUMED_DURATION_SECONDS = 5.0;

    /**
     * This list defines OpModes that should be selectable by the user. This will then
     * be used to determine your tasks in {@link #onReady(Reference, Controls)}.
     * For example, you may have configurations for RED_LEFT, RED_RIGHT, BLUE_LEFT, BLUE_RIGHT.
     * By default, this will be empty, and the user will not be prompted for a selection.
     *
     * @see #setOpModes(Object...)
     */
    private final ArrayList<Reference<?>> opModes = new ArrayList<>();
    private final ConcurrentLinkedDeque<RobotTask> tasks = new ConcurrentLinkedDeque<>();
    // Pre and post queues cannot have their tasks removed, so we can rely on their .size() methods
    private final ArrayDeque<RobotTask> postQueue = new ArrayDeque<>();
    private final ArrayDeque<RobotTask> preQueue = new ArrayDeque<>();
    @NonNull
    private HashSet<BunyipsSubsystem> updatedSubsystems = new HashSet<>();
    private int taskCount;
    private UserSelection<Reference<?>> userSelection;
    private int currentTask = 1;
    private volatile boolean callbackReceived;
    private boolean safeToAddTasks;
    private boolean hardwareStopOnFinish = true;

    private void callback(@Nullable Reference<?> selectedOpMode) {
        if (isStopRequested())
            return;
        safeToAddTasks = true;
        try {
            onReady(selectedOpMode, userSelection != null ? userSelection.getSelectedButton() : null);
        } catch (Exception e) {
            Exceptions.handle(e, telemetry::log);
        }
        // Add any queued tasks
        for (RobotTask task : postQueue) {
            addTask(task);
        }
        for (RobotTask task : preQueue) {
            addTaskFirst(task);
        }
        preQueue.clear();
        postQueue.clear();
        String timeLeft = getApproximateTimeLeft();
        Text.Builder out = Text.builder();
        out.append("[AutonomousBunyipsOpMode] onReady() called | %% task(s) queued%\n",
                userSelection != null ? "usr: " + selectedOpMode + " | " : "",
                taskCount,
                timeLeft.isEmpty() ? "" : timeLeft + " to complete"
        );
        for (RobotTask task : tasks) {
            if (!(task instanceof Task)) continue;
            out.append("   -> %\n", ((Task) task).toVerboseString());
        }
        Dbg.logd(out.toString());
        callbackReceived = true;
    }

    @Override
    protected final void onInit() {
        // Run user-defined hardware initialisation
        try {
            onInitialise();
        } catch (Exception e) {
            Exceptions.handle(e, telemetry::log);
        }
        if (updatedSubsystems.isEmpty())
            updatedSubsystems = BunyipsSubsystem.instances;
        // Convert user defined OpModeSelections to varargs
        Reference<?>[] varargs = opModes.toArray(new Reference[0]);
        if (varargs.length == 0) {
            opModes.add(Reference.empty());
        }
        if (varargs.length > 1) {
            // Run task allocation if OpModeSelections are defined
            // This will run asynchronously, and the callback will be called
            // when the user has selected an OpMode
            userSelection = new UserSelection<>(this, this::callback, varargs);
            Threads.start(userSelection);
        } else {
            // There are no OpMode selections, so just run the callback with the default OpMode
            callback(opModes.get(0));
        }
    }

    /**
     * Perform one time operations after start is pressed.
     * Unlike {@link #onInitDone}, this will only execute once play is hit and not when initialisation is done.
     * <p>
     * If overriding this method, it is strongly recommended to call {@code super.onStart()} in your method to
     * ensure that the asynchronous task allocation has been notified to stop immediately. This is
     * not required if {@link #setOpModes(Object...)} returns null.
     */
    @Override
    protected void onStart() {
        if (isStopRequested())
            return;
        if (userSelection != null) {
            // UserSelection will internally check opMode.isInInit() to see if it should terminate itself
            // but we should wait here until it has actually terminated
            Threads.waitFor(userSelection, true);
        }
        // Busy wait here until onReady() has processed and the callback is fully joined
        // This is safe to do as there are no main thread operations left to run
        while (!callbackReceived) {
            sleep(1);
        }
    }

    @Override
    protected final void activeLoop() {
        // Run any code defined by the user
        try {
            periodic();
        } catch (Exception e) {
            Exceptions.handle(e, telemetry::log);
        }

        // Run the queue of tasks
        synchronized (tasks) {
            RobotTask currentTask = tasks.peekFirst();
            if (currentTask == null) {
                telemetry.log("<font color='gray'>auto:</font> tasks done -> finishing");
                finish(hardwareStopOnFinish);
                return;
            }

            telemetry.setOverheadSubtitle(
                    formatString("<small><font color='aqua'>Running task <b>%/%</b></font> | %%</small>",
                            this.currentTask, taskCount, currentTask, getApproximateTimeLeft())
            );

            try {
                // AutonomousBunyipsOpMode is handling all task completion checks, manual checks not required
                if (currentTask.pollFinished()) {
                    tasks.removeFirst();
                    double runTime = 0;
                    if (currentTask instanceof Task) {
                        runTime = ((Task) currentTask).getDeltaTime().in(Seconds);
                    }
                    Dbg.logd("[AutonomousBunyipsOpMode] task %/% (%) finished%", this.currentTask, taskCount, currentTask, runTime != 0 ? " -> " + runTime + "s" : "");
                    this.currentTask++;
                }

                currentTask.run();
            } catch (Exception e) {
                Exceptions.handle(e, telemetry::log);
            }
        }

        // Update all subsystems
        for (BunyipsSubsystem subsystem : updatedSubsystems) {
            try {
                subsystem.update();
            } catch (Exception e) {
                Exceptions.handle(e, telemetry::log);
            }
        }
    }

    /**
     * Use an init task if you wish to run looping code during the initialisation phase of the OpMode.
     *
     * @see #setInitTask
     */
    @Override
    protected final boolean onInitLoop() {
        return userSelection == null || !Threads.isRunning(userSelection);
    }

    /**
     * Call to disable the automatic stopping of the hardware when the OpMode finishes after no tasks are left.
     * This does not impact the automated stopping of the hardware when the OpMode is requested to stop.
     */
    public void disableHardwareStopOnFinish() {
        hardwareStopOnFinish = false;
    }

    /**
     * Call to manually add the subsystems that should be managed by AutonomousBunyipsOpMode. Using this method will override
     * the automatic collection of {@link BunyipsSubsystem}s, and allows you to determine which subsystems will be managed for
     * this OpMode.
     * <p>
     * For most cases, using this method is not required and all you need to do is construct your subsystems and they
     * will be managed automatically. This method is for advanced cases where you don't want this behaviour to happen.
     *
     * @param subsystems the restrictive list of subsystems to be managed and updated by ABOM
     */
    public final void useSubsystems(BunyipsSubsystem... subsystems) {
        if (!NullSafety.assertNotNull(Arrays.stream(subsystems).toArray())) {
            throw new RuntimeException("Null subsystems were added in the addSubsystems() method!");
        }
        Collections.addAll(updatedSubsystems, subsystems);
    }

    /**
     * Can be called to add custom {@link Task}s in a robot's autonomous
     *
     * @param newTask task to add to the run queue
     * @param ack     suppress the warning that a task was added manually before onReady
     */
    public final void addTask(@NotNull RobotTask newTask, boolean ack) {
        checkTaskForDependency(newTask);
        if (!safeToAddTasks && !ack) {
            telemetry.log("<font color='gray'>auto:</font> <font color='yellow'>caution!</font> a task was added manually before the onReady callback");
        }
        synchronized (tasks) {
            tasks.add(newTask);
        }
        if (newTask instanceof TaskGroup)
            ((TaskGroup) newTask).logCreation();
        taskCount++;
        telemetry.log("<font color='gray'>auto:</font> %<i>(t=%)</i> -> added %/%", newTask, getTaskTimeout(newTask), taskCount, taskCount);
    }

    /**
     * Can be called to add custom {@link Task}s in a robot's autonomous
     *
     * @param newTask task to add to the run queue
     */
    public final void addTask(@NotNull RobotTask newTask) {
        addTask(newTask, false);
    }

    /**
     * Implicitly construct a new {@link RunTask} and add it to the run queue
     *
     * @param runnable the code to add to the run queue to run once
     */
    public final void addTask(@NotNull Runnable runnable) {
        addTask(new RunTask(runnable));
    }

    /**
     * Implicitly construct a new {@link RunTask} and add it to the run queue with a custom name
     *
     * @param runnable the code to add to the run queue to run once
     * @param name     the name of the task
     */
    public final void addTask(@NotNull Runnable runnable, String name) {
        addTask(new RunTask(runnable).withName(name));
    }

    /**
     * Insert a task at a specific index in the queue. This is useful for adding tasks that should be run
     * at a specific point in the autonomous sequence. Note that this function immediately produces side effects,
     * and subsequent calls will not be able to insert tasks at the same index due to the shifting of tasks.
     *
     * @param index   the index to insert the task at, starting from 0
     * @param newTask the task to add to the run queue
     */
    public final void addTaskAtIndex(int index, @NotNull RobotTask newTask) {
        checkTaskForDependency(newTask);
        ArrayDeque<RobotTask> tmp = new ArrayDeque<>();
        synchronized (tasks) {
            if (index < 0 || index > tasks.size())
                throw new IllegalArgumentException("Cannot insert task at index " + index + ", out of bounds");
            // Deconstruct the queue to insert the new task
            while (tasks.size() > index) {
                tmp.add(tasks.removeLast());
            }
            // Insert the new task
            tasks.add(newTask);
            // Refill the queue
            while (!tmp.isEmpty()) {
                tasks.add(tmp.removeLast());
            }
        }
        if (newTask instanceof TaskGroup)
            ((TaskGroup) newTask).logCreation();
        taskCount++;
        telemetry.log("<font color='gray'>auto:</font> %<i>(t=%)</i> -> inserted %/%", newTask, getTaskTimeout(newTask), index, taskCount);
    }

    /**
     * Insert a task at a specific index in the queue. This is useful for adding tasks that should be run
     * at a specific point in the autonomous sequence. Note that this function immediately produces side effects,
     * and subsequent calls will not be able to insert tasks at the same index due to the shifting of tasks.
     *
     * @param index    the index to insert the task at, starting from 0
     * @param runnable the code to add to the run queue to run once
     */
    public final void addTaskAtIndex(int index, @NotNull Runnable runnable) {
        addTaskAtIndex(index, new RunTask(runnable));
    }

    /**
     * Add a task to the run queue, but after {@link #onReady(Reference, Controls)} has processed tasks. This is useful
     * to call when working with tasks that should be queued at the very end of the autonomous, while still
     * being able to add tasks asynchronously with user input in {@link #onReady(Reference, Controls)}.
     *
     * @param newTask task to add to the run queue
     */
    public final void addTaskLast(@NotNull RobotTask newTask) {
        checkTaskForDependency(newTask);
        if (!callbackReceived) {
            postQueue.add(newTask);
            telemetry.log("<font color='gray'>auto:</font> %<i>(t=%)</i> -> queued end-init %/%", newTask, getTaskTimeout(newTask), postQueue.size(), postQueue.size());
            return;
        }
        synchronized (tasks) {
            tasks.addLast(newTask);
        }
        if (newTask instanceof TaskGroup)
            ((TaskGroup) newTask).logCreation();
        taskCount++;
        telemetry.log("<font color='gray'>auto:</font> %<i>(t=%)</i> -> added %/%", newTask, getTaskTimeout(newTask), taskCount, taskCount);
    }

    /**
     * Add a task to the very start of the queue. This is useful to call when working with tasks that
     * should be queued at the very start of the autonomous, while still being able to add tasks
     * asynchronously with user input in {@link #onReady(Reference, Controls)}.
     *
     * @param newTask task to add to the run queue
     */
    public final void addTaskFirst(@NotNull RobotTask newTask) {
        checkTaskForDependency(newTask);
        if (!callbackReceived) {
            preQueue.add(newTask);
            telemetry.log("<font color='gray'>auto:</font> %<i>(t=%)</i> -> queued end-init 1/%", newTask, getTaskTimeout(newTask), preQueue.size());
            return;
        }
        synchronized (tasks) {
            tasks.addFirst(newTask);
        }
        if (newTask instanceof TaskGroup)
            ((TaskGroup) newTask).logCreation();
        taskCount++;
        telemetry.log("<font color='gray'>auto:</font> %<i>(t=%)</i> -> added 1/%", newTask, getTaskTimeout(newTask), taskCount);
    }

    private String getTaskTimeout(RobotTask task) {
        if (task instanceof Task) {
            // Try to extract the timeout of the task, as RobotTask does not have a timeout
            Measure<Time> timeout = ((Task) task).getTimeout();
            // INFINITE_TASK is defined as Seconds.zero()
            return timeout.magnitude() != 0.0
                    ? round(timeout.in(Seconds), 1) + "s"
                    : "∞";
        }
        return "?";
    }

    private String getApproximateTimeLeft() {
        // Must use an atomic boolean due to lambda restrictions
        AtomicBoolean approx = new AtomicBoolean(false);
        // Attempt to get the time left for all tasks by summing their timeouts
        double timeLeft = tasks.stream().mapToDouble(task -> {
            // We cannot extract the duration of a task that is not a Task, we will return zero instead of the assumption
            // as they are completely out of our control and we don't even know how they function
            if (!(task instanceof Task)) return 0;
            Measure<Time> timeout = ((Task) task).getTimeout();
            // We have to approximate and guess as we cannot determine the duration of a task that is infinite
            if (timeout.magnitude() == 0.0) {
                // We also adjust the approx flag so we can notify the user that the time is an estimate with a tilde
                approx.set(true);
                return INFINITE_TASK_ASSUMED_DURATION_SECONDS;
            }
            return timeout.in(Seconds);
        }).sum();
        // Determine the time left for all tasks
        RobotTask curr = tasks.peekFirst();
        if (curr instanceof Task) {
            Measure<Time> timeout = ((Task) curr).getDeltaTime();
            // Offset by the current task's time left to interpolate between tasks
            timeLeft -= timeout.in(Seconds);
        } else {
            // This task is low resolution and we cannot determine the time left, we'll let the user know that the
            // time is now inaccurate
            approx.set(true);
        }
        // If we get negative time, our guess was very wrong so we'll return a blank string
        return timeLeft > 0 ? " | " + (approx.get() ? "~" : "") + round(timeLeft, 1) + "s" : "";
    }

    /**
     * Removes whatever task is at the given queue position
     * Note: this will remove the index and shift all other tasks down, meaning that
     * tasks being added/removed will affect the index of the task you want to remove
     *
     * @param taskIndex the array index to be removed, starting from 0
     */
    public final void removeTaskAtIndex(int taskIndex) {
        synchronized (tasks) {
            if (taskIndex < 0 || taskIndex >= tasks.size())
                throw new IllegalArgumentException("Cannot remove task at index " + taskIndex + ", out of bounds");

            /*
             * In the words of the great Lucas Bubner:
             *      You've made an iterator for all those tasks
             *      which is the goofinator car that can drive around your array
             *      calling .next() on your car will move it one down the array
             *      then if you call .remove() on your car it will remove the element wherever it is
             */
            Iterator<RobotTask> iterator = tasks.iterator();

            int counter = 0;
            while (iterator.hasNext()) {
                iterator.next();

                if (counter == taskIndex) {
                    iterator.remove();
                    telemetry.log("<font color='gray'>auto:</font> task at index % -> removed", taskIndex);
                    taskCount--;
                    break;
                }
                counter++;
            }
        }
    }

    /**
     * Remove a task from the queue
     * This assumes that the overhead OpMode has instance control over the task, as this method
     * will search for an object reference to the task and remove it from the queue
     *
     * @param task the task to be removed
     */
    public final void removeTask(@NotNull RobotTask task) {
        synchronized (tasks) {
            if (tasks.contains(task)) {
                tasks.remove(task);
                telemetry.log("<font color='gray'>auto:</font> task %<i>(t=%)</i> -> removed", task, getTaskTimeout(task));
                taskCount--;
            } else {
                telemetry.log("<font color='gray'>auto:</font> task %<i>(t=%)</i> -> <font color='yellow'>not found</font>", task, getTaskTimeout(task));
            }
        }
    }

    /**
     * Removes the last task in the task queue
     */
    public final void removeTaskLast() {
        synchronized (tasks) {
            tasks.removeLast();
        }
        taskCount--;
        telemetry.log("<font color='gray'>auto:</font> task at index % -> removed", taskCount + 1);
    }

    /**
     * Removes the first task in the task queue
     */
    public final void removeTaskFirst() {
        synchronized (tasks) {
            tasks.removeFirst();
        }
        taskCount--;
        telemetry.log("<font color='gray'>auto:</font> task at index 0 -> removed");
    }

    private void checkTaskForDependency(RobotTask task) {
        if (task instanceof Task) {
            ((Task) task).getDependency().ifPresent((s) -> {
                if (!updatedSubsystems.contains(s))
                    Dbg.warn(getClass(), "Task % has a dependency on %, but it is not being updated by the AutonomousBunyipsOpMode. Please ensure it is being updated properly through addSubsystems().", task, s);
            });
        }
    }

    /**
     * Runs upon the pressing of the INIT button on the Driver Station.
     * This is where your hardware should be initialised. You may also add specific tasks to the queue
     * here, but it is recommended to use {@link #setInitTask(RobotTask)} or {@link #onReady(Reference, Controls)} instead.
     */
    protected abstract void onInitialise();

    /**
     * Call to define your OpModeSelections, if you list any, then the user will be prompted to select
     * an OpMode before the OpMode begins. If you return null, then the user will not
     * be prompted for a selection, and the OpMode will move to task-ready state immediately.
     * <pre>{@code
     *     setOpModes(
     *             "GO_PARK",
     *             "GO_SHOOT",
     *             "GO_SHOOT_AND_PARK",
     *             "SABOTAGE_ALLIANCE"
     *     );
     *     // Use `StartingPositions.use();` for using the four Robot starting positions
     * }</pre>
     */
    protected final void setOpModes(@Nullable List<Object> selectableOpModes) {
        if (selectableOpModes == null) return;
        setOpModes(selectableOpModes.toArray(new Object[0]));
    }


    /**
     * Call to define your OpModeSelections, if you list any, then the user will be prompted to select
     * an OpMode before the OpMode begins. If you return null, then the user will not
     * be prompted for a selection, and the OpMode will move to task-ready state immediately.
     * <pre>{@code
     *     setOpModes(
     *             "GO_PARK",
     *             "GO_SHOOT",
     *             "GO_SHOOT_AND_PARK",
     *             "SABOTAGE_ALLIANCE"
     *     );
     *     // Use `StartingPositions.use();` for using the four Robot starting positions
     * }</pre>
     */
    protected final void setOpModes(@Nullable Object... selectableOpModes) {
        if (selectableOpModes == null) return;
        opModes.clear();
        for (Object selectableOpMode : selectableOpModes) {
            if (selectableOpMode instanceof Reference<?>) {
                opModes.add((Reference<?>) selectableOpMode);
            } else {
                opModes.add(new Reference<>(selectableOpMode));
            }
        }
    }

    /**
     * Called when the OpMode is ready to process tasks.
     * This will happen when the user has selected an OpMode, or if {@link #setOpModes(Object...)} returned null,
     * in which case it will run immediately after {@code static_init} has completed.
     * This is where you should add your tasks to the run queue.
     *
     * @param selectedOpMode the OpMode selected by the user, if applicable. Will be NULL if the user does not select an OpMode (and OpModes were available).
     *                       Will be an empty reference if {@link #setOpModes(Object...)} returned null (no OpModes to select).
     * @param selectedButton the button selected by the user. Will be Controls.NONE if no selection is made or given.
     * @see #addTask(RobotTask)
     */
    protected abstract void onReady(@Nullable Reference<?> selectedOpMode, Controls selectedButton);

    /**
     * Override to this method to add extra code to the activeLoop, which will be run before
     * the task queue is processed.
     */
    protected void periodic() {
    }
}
