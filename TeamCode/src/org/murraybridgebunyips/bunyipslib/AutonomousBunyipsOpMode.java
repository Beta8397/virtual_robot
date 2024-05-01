package org.murraybridgebunyips.bunyipslib;

import androidx.annotation.Nullable;

import org.jetbrains.annotations.NotNull;
import org.murraybridgebunyips.bunyipslib.tasks.RunTask;
import org.murraybridgebunyips.bunyipslib.tasks.bases.RobotTask;
import org.murraybridgebunyips.bunyipslib.tasks.bases.Task;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.HashSet;
import java.util.Iterator;
import java.util.List;

/**
 * {@link BunyipsOpMode} variant for Autonomous operation. Uses the {@link Task} system for a queued action OpMode.
 *
 * @author Lucas Bubner, 2023
 * @author Lachlan Paul, 2023
 * @see RoadRunner
 * @see BunyipsOpMode
 */
public abstract class AutonomousBunyipsOpMode extends BunyipsOpMode {

    /**
     * This list defines OpModes that should be selectable by the user. This will then
     * be used to determine your tasks in {@link #onReady(OpModeSelection)}.
     * For example, you may have configurations for RED_LEFT, RED_RIGHT, BLUE_LEFT, BLUE_RIGHT.
     * By default, this will be empty, and the user will not be prompted for a selection.
     *
     * @see #setOpModes(Object...)
     */
    private final ArrayList<OpModeSelection> opModes = new ArrayList<>();
    private final ArrayDeque<RobotTask> tasks = new ArrayDeque<>();
    // Pre and post queues cannot have their tasks removed, so we can rely on their .size() methods
    private final ArrayDeque<RobotTask> postQueue = new ArrayDeque<>();
    private final ArrayDeque<RobotTask> preQueue = new ArrayDeque<>();
    private final HashSet<BunyipsSubsystem> updatedSubsystems = new HashSet<>();
    private int taskCount;
    private UserSelection<OpModeSelection> userSelection;
    // Init-task does not count as a queued task, so we start at 1
    private int currentTask = 1;
    private RobotTask initTask;
    private boolean hasGottenCallback;

    private void callback(@Nullable OpModeSelection selectedOpMode) {
        hasGottenCallback = true;
        if (selectedOpMode != null) {
            log("auto: mode selected. running opmode " + selectedOpMode);
        } else {
            log("auto: mode selected. running default opmode");
        }
        onReady(selectedOpMode);
        // Add any queued tasks
        for (RobotTask task : postQueue) {
            addTask(task);
        }
        for (RobotTask task : preQueue) {
            addTaskFirst(task);
        }
        preQueue.clear();
        postQueue.clear();
    }

    @Override
    protected final void onInit() {
        // Run user-defined hardware initialisation
        onInitialise();
        if (initTask == null) {
            log("auto: initTask is null, skipping.");
        }
        // Convert user defined OpModeSelections to varargs
        OpModeSelection[] varargs = opModes.toArray(new OpModeSelection[0]);
        if (varargs.length == 0) {
            log("auto: no setOpModes() defined, skipping selection phase");
            opModes.add(new OpModeSelection(new UserDefaultSelection()));
        }
        if (varargs.length > 1) {
            // Run task allocation if OpModeSelections are defined
            // This will run asynchronously, and the callback will be called
            // when the user has selected an OpMode
            log("auto: waiting for user input...");
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
        if (userSelection != null) {
            // UserSelection will internally check opMode.isInInit() to see if it should terminate itself
            // but we should wait here until it has actually terminated
            Threads.waitFor(userSelection, true);
        }
    }

    @Override
    protected final void activeLoop() {
        if (!hasGottenCallback) {
            // Not ready to run tasks yet, we can't do much. This shouldn't really happen,
            // but just in case, we'll log it and wait for the callback to be run
            Dbg.logd("AutonomousBunyipsOpMode is busy-waiting for a late UserSelection callback...");
            return;
        }

        // Run any code defined by the user
        periodic();

        // Run the queue of tasks
        RobotTask currentTask = tasks.peekFirst();
        if (currentTask == null) {
            log("auto: all tasks done, finishing...");
            finish();
            return;
        }

        addTelemetry("Running task (%/%): %", this.currentTask, taskCount, currentTask);

        // AutonomousBunyipsOpMode is handling all task completion checks, manual checks not required
        if (currentTask.pollFinished()) {
            tasks.removeFirst();
            log("auto: task %/% (%) finished", this.currentTask, taskCount, currentTask);
            this.currentTask++;
        }

        currentTask.run();

        // Update all subsystems
        for (BunyipsSubsystem subsystem : updatedSubsystems) {
            subsystem.update();
        }
    }

    /**
     * Run code in a loop AFTER {@link #onInitialise()} has completed, until
     * start is pressed on the Driver Station or the {@link #setInitTask initTask} is done.
     * If not implemented, the OpMode will try to run your initTask, and if that is null,
     * the {@code dynamic_init} phase will be skipped.
     * This method should not be overridden,
     * if you need to do anything in the init-loop use {@link #setInitTask(RobotTask)}.
     *
     * @see #setInitTask
     */
    @Override
    protected final boolean onInitLoop() {
        if (initTask != null) {
            initTask.run();
            return initTask.pollFinished() && (userSelection == null || !Threads.isRunning(userSelection));
        }
        return userSelection == null || !Threads.isRunning(userSelection);
    }

    /**
     * Call to add subsystems that should be updated by AutonomousBunyipsOpMode. This is required to be called
     * as some subsystems rely on their {@code update()} method to dispatch tasks.
     *
     * @param subsystems the subsystems to be periodically called for their {@code update()} method.
     */
    public final void addSubsystems(BunyipsSubsystem... subsystems) {
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
        if (!hasGottenCallback && !ack) {
            log("auto: caution! a task was added manually before the onReady callback");
        }
        tasks.add(newTask);
        taskCount++;
        log("auto: % has been added as task %/%", newTask, taskCount, taskCount);
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
     * Add a task to the run queue, but after {@link #onReady(OpModeSelection)} has processed tasks. This is useful
     * to call when working with tasks that should be queued at the very end of the autonomous, while still
     * being able to add tasks asynchronously with user input in {@link #onReady(OpModeSelection)}.
     *
     * @param newTask task to add to the run queue
     */
    public final void addTaskLast(@NotNull RobotTask newTask) {
        checkTaskForDependency(newTask);
        if (!hasGottenCallback) {
            postQueue.add(newTask);
            log("auto: % has been queued as end-init task %/%", newTask, postQueue.size(), postQueue.size());
            return;
        }
        tasks.addLast(newTask);
        taskCount++;
        log("auto: % has been added as task %/%", newTask, taskCount, taskCount);
    }

    /**
     * Add a task to the very start of the queue. This is useful to call when working with tasks that
     * should be queued at the very start of the autonomous, while still being able to add tasks
     * asynchronously with user input in {@link #onReady(OpModeSelection)}.
     *
     * @param newTask task to add to the run queue
     */
    public final void addTaskFirst(@NotNull RobotTask newTask) {
        checkTaskForDependency(newTask);
        if (!hasGottenCallback) {
            preQueue.add(newTask);
            log("auto: % has been queued as end-init task 1/%", newTask, preQueue.size());
            return;
        }
        tasks.addFirst(newTask);
        taskCount++;
        log("auto: % has been added as task 1/%", newTask, taskCount);
    }

    /**
     * Removes whatever task is at the given queue position
     * Note: this will remove the index and shift all other tasks down, meaning that
     * tasks being added/removed may affect the index of the task you want to remove
     *
     * @param taskIndex the array index to be removed
     */
    public final void removeTaskIndex(int taskIndex) {
        if (taskIndex < 0) {
            throw new IllegalArgumentException("Auto: Cannot remove items starting from last index, this isn't Python");
        }

        if (taskIndex > tasks.size()) {
            throw new IllegalArgumentException("Auto: Given index exceeds array size");
        }

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
            if (counter == taskIndex) {
                iterator.remove();
                log("auto: task at index % was removed", taskIndex);
                taskCount--;
                break;
            }

            iterator.next();
            counter++;
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
        if (tasks.contains(task)) {
            tasks.remove(task);
            log("auto: task % was removed", task);
            taskCount--;
        } else {
            log("auto: task % was not found in the queue", task);
        }
    }

    /**
     * Removes the last task in the task queue
     */
    public final void removeTaskLast() {
        tasks.removeLast();
        taskCount--;
        log("auto: task at index % was removed", taskCount + 1);
    }

    /**
     * Removes the first task in the task queue
     */
    public final void removeTaskFirst() {
        tasks.removeFirst();
        taskCount--;
        log("auto: task at index 0 was removed");
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
     * here, but it is recommended to use {@link #setInitTask(RobotTask)} or {@link #onReady(OpModeSelection)} instead.
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
            if (selectableOpMode instanceof OpModeSelection) {
                opModes.add((OpModeSelection) selectableOpMode);
            } else {
                opModes.add(new OpModeSelection(selectableOpMode));
            }
        }
    }

    /**
     * Set a task that will run as an init-task. This will run
     * after your {@link #onInitialise()} has completed, allowing you to initialise hardware first.
     * This is an optional method.
     * <p>
     * You should store any running variables inside the task itself, and keep the instance of the task
     * defined as a field in your OpMode. You can then use this value in your {@link #onInitDone()} to do
     * what you need to after the init-task has finished. This method should be paired with {@link #onInitDone()}
     * to do anything after the initTask has finished.
     * </p>
     * If you do not define an initTask by returning null, then the init-task {@code dynamic_init} phase will be skipped.
     *
     * @see #onInitDone()
     * @see #addTaskFirst(RobotTask)
     * @see #addTaskLast(RobotTask)
     */
    protected final void setInitTask(@Nullable RobotTask task) {
        if (initTask != null) {
            Dbg.warn(getClass(), "Init-task has already been set to %, overriding it with %...", initTask, task);
        }
        initTask = task;
    }

    /**
     * Called when the OpMode is ready to process tasks.
     * This will happen when the user has selected an OpMode, or if {@link #setOpModes(Object...)} returned null,
     * in which case it will run immediately after {@code static_init} has completed.
     * This is where you should add your tasks to the run queue.
     *
     * @param selectedOpMode the OpMode selected by the user, if applicable. Will be UserDefaultSelection if no OpModeSelections were defined, or
     *                       NULL if the user did not select an OpMode.
     * @see #addTask(RobotTask)
     */
    protected abstract void onReady(@Nullable OpModeSelection selectedOpMode);

    /**
     * Override to this method to add extra code to the activeLoop, which will be run before
     * the task queue is processed.
     */
    protected void periodic() {
    }
}
