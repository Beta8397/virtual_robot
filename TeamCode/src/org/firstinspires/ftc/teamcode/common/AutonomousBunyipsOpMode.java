package org.firstinspires.ftc.teamcode.common;

import androidx.annotation.Nullable;
import kotlin.Unit;
import org.firstinspires.ftc.teamcode.common.tasks.AutoTask;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

/**
 * OpMode abstraction for Autonomous operation using the BunyipsOpMode ecosystem.
 *
 * @author Lucas Bubner, 2023
 * @author Lachlan Paul, 2023
 */

public abstract class AutonomousBunyipsOpMode extends BunyipsOpMode {

    /**
     * This list defines OpModes that should be selectable by the user. This will then
     * be used to determine your tasks in {@link #onQueueReady(OpModeSelection)}.
     * For example, you may have configurations for RED_LEFT, RED_RIGHT, BLUE_LEFT, BLUE_RIGHT.
     * By default, this will be empty, and the user will not be prompted for a selection.
     *
     * @see #setOpModes()
     */
    protected final ArrayList<OpModeSelection> opModes = new ArrayList<>();
    private final ArrayDeque<AutoTask> tasks = new ArrayDeque<>();
    private final ArrayDeque<AutoTask> postQueue = new ArrayDeque<>();
    private final ArrayDeque<AutoTask> preQueue = new ArrayDeque<>();
    private UserSelection<OpModeSelection> userSelection;
    private int currentTask = 1;
    private AutoTask initTask;
    private boolean hasGottenCallback;

    private Unit callback(@Nullable OpModeSelection selectedOpMode) {
        hasGottenCallback = true;
        if (selectedOpMode != null) {
            log("auto: mode selected. running opmode " + selectedOpMode.getName());
        } else {
            log("auto: mode selected. running default opmode");
        }
        // Interface Unit to be void
        onQueueReady(selectedOpMode);
        // Add any queued tasks
        tasks.addAll(postQueue);
        for (AutoTask task : preQueue) {
            tasks.addFirst(task);
        }
        return Unit.INSTANCE;
    }

    @Override
    protected final void onInit() {
        // Run user-defined hardware initialisation
        onInitialisation();
        // Set user-defined initTask
        initTask = setInitTask();
        if (initTask == null) {
            log("auto: initTask is null, skipping.");
        }
        List<OpModeSelection> userSelections = setOpModes();
        if (userSelections != null) {
            // User defined OpModeSelections
            opModes.addAll(userSelections);
        }
        // Convert user defined OpModeSelections to varargs
        OpModeSelection[] varargs = opModes.toArray(new OpModeSelection[0]);
        if (varargs.length == 0) {
            log("auto: no OpModeSelections defined, skipping selection phase");
            opModes.add(new OpModeSelection("DEFAULT"));
        }
        if (varargs.length > 1) {
            // Run task allocation if OpModeSelections are defined
            // This will run asynchronously, and the callback will be called
            // when the user has selected an OpMode
            log("auto: waiting for user input...");
            userSelection = new UserSelection<>(this, this::callback, varargs);
            userSelection.start();
        } else {
            // There are no OpMode selections, so just run the callback with the default OpMode
            callback(opModes.get(0));
        }
    }

    /**
     * Perform one time operations after start is pressed.
     * Unlike onInitDone, this will only execute once play is hit and not when initialisation is done.
     * <p>
     * If overriding this method, it is strongly recommended to call `super.onStart()` in your method to
     * ensure that the asynchronous task allocation has been notified to stop immediately. This is
     * not required if setOpModes() returns null.
     */
    @Override
    protected void onStart() {
        if (userSelection != null) {
            // UserSelection will internally check opMode.isInInit() to see if it should terminate itself
            // automatically, but this is to ensure that the thread receives the message immediately
            // upon start as user input is now impossible to retrieve and we need a callback ASAP
            userSelection.interrupt();
        }
    }

    @Override
    protected final void activeLoop() {
        // Run any code defined by the user
        onActiveLoop();
        if (!hasGottenCallback) {
            // Not ready to run tasks yet, tell the user selection to terminate if it hasn't
            if (!userSelection.isInterrupted())
                userSelection.interrupt();
            return;
        }
        // Run the queue of tasks
        AutoTask currentTask = tasks.peekFirst();

        if (currentTask == null) {
            log("auto: all tasks done, finishing...");
            finish();
            return;
        }

        // Why does it have to be like this
        addTelemetry("Running task (%/%): %", this.currentTask, tasks.size(), currentTask.getClass().getSimpleName());

        currentTask.run();

        // AutonomousBunyipsOpMode is handling all task completion checks, manual checks not required
        if (currentTask.isFinished()) {
            tasks.removeFirst();
            log("auto: task %/% (%) finished", this.currentTask, tasks.size() + 1, currentTask.getClass().getSimpleName());
            this.currentTask++;
        }
    }

    /**
     * Run code in a loop AFTER onBegin() has completed, until
     * start is pressed on the Driver Station or the {@link #setInitTask initTask} is done.
     * If not implemented, the opMode will try to run your initTask, and if that is null,
     * the dynamic_init phase will be skipped.
     * Overriding this method will fully detach your UserSelection from alerting BYO of its runtime,
     * so override with caution or ensure to use a super call.
     *
     * @see #setInitTask
     */
    @Override
    protected boolean onInitLoop() {
        if (initTask != null) {
            initTask.run();
            return initTask.isFinished() && (userSelection == null || !userSelection.isAlive());
        }
        return userSelection == null || !userSelection.isAlive();
    }

    /**
     * Can be called to add custom tasks in a robot's autonomous
     *
     * @param newTask task to add to the run queue
     */
    public void addTask(AutoTask newTask) {
        if (!hasGottenCallback) {
            log("auto: caution! a task was added manually before the onReady callback");
        }
        tasks.add(newTask);
        log("auto: % has been added as task %/%", newTask.getClass().getSimpleName(), tasks.size(), tasks.size());
    }

    /**
     * Add a task to the run queue, but after onReady() has processed tasks. This is useful to call
     * when working with tasks that should be queued at the very end of the autonomous, while still
     * being able to add tasks asynchronously with user input in onReady().
     */
    public void addTaskLast(AutoTask newTask) {
        if (!hasGottenCallback) {
            postQueue.add(newTask);
            log("auto: % has been queued as end-init task %/%", newTask.getClass().getSimpleName(), postQueue.size(), postQueue.size());
            return;
        }
        tasks.addLast(newTask);
        log("auto: % has been added as task %/%", newTask.getClass().getSimpleName(), tasks.size(), tasks.size());
    }

    /**
     * Add a task to the very start of the queue. This is useful to call when working with tasks that
     * should be queued at the very start of the autonomous, while still being able to add tasks
     * asynchronously with user input in onReady().
     */
    public void addTaskFirst(AutoTask newTask) {
        if (!hasGottenCallback) {
            preQueue.add(newTask);
            log("auto: % has been queued as end-init task 1/%", newTask.getClass().getSimpleName(), preQueue.size());
            return;
        }
        tasks.addFirst(newTask);
        log("auto: % has been added as task 1/%", newTask.getClass().getSimpleName(), tasks.size());
    }

    /**
     * Removes whatever task is at the given array index
     *
     * @param taskIndex the array index to be removed (did I really need to tell you?)
     */
    public void removeTaskIndex(int taskIndex) {
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
        Iterator<AutoTask> iterator = tasks.iterator();

        int counter = 0;
        while (iterator.hasNext()) {
            if (counter == taskIndex) {
                iterator.remove();
                log("auto: task at index % was removed", taskIndex);
                break;
            }

            iterator.next();
            counter++;
        }
    }

    /**
     * Removes the last task in the task queue
     */
    public void removeTaskLast() {
        tasks.removeLast();
        log("auto: task at index % was removed", tasks.size() + 1);
    }

    /**
     * Removes the first task in the task queue
     */
    public void removeTaskFirst() {
        tasks.removeFirst();
        log("auto: task at index 0 was removed");
    }


    /**
     * Runs upon the pressing of the INIT button on the Driver Station.
     * This is where your hardware should be initialised. You may also add specific tasks to the queue
     * here, but it is recommended to use {@link #setInitTask()} or {@link #onQueueReady(OpModeSelection)} instead.
     *
     * @see #onInit()
     */
    protected abstract void onInitialisation();

    /**
     * Implement to define your OpModeSelections, if you list any, then the user will be prompted to select
     * an OpMode before the OpMode begins. If you return null, then the user will not
     * be prompted for a selection, and the OpMode will move to task-ready state immediately.
     * <pre><code>
     *     protected List<OpModeSelection> setOpModes() {
     *         return Arrays.asList(
     *                 new OpModeSelection("LEFT_BLUE"),
     *                 new OpModeSelection("RIGHT_BLUE"),
     *                 new OpModeSelection("LEFT_RED"),
     *                 new OpModeSelection("RIGHT_RED")
     *         );
     *     }
     * </code></pre>
     */
    protected abstract List<OpModeSelection> setOpModes();

    /**
     * Return a task that will run as an init-task. This will run
     * after your onInitialisation() has completed, allowing you to initialise hardware first.
     * This is an optional method.
     * <p>
     * You should store any running variables inside the task itself, and keep the instance of the task
     * defined as a field in your OpMode. You can then use this value in your onInitDone() to do
     * what you need to after the init-task has finished. This method should be paired with {@link #onInitDone()}
     * to do anything after the initTask has finished.
     * </p>
     * If you do not define an initTask by returning null, then the init-task (dynamic_init) phase will be skipped.
     *
     * @see #onInitDone()
     * @see #addTaskFirst(AutoTask)
     * @see #addTaskLast(AutoTask)
     */
    protected abstract AutoTask setInitTask();

    /**
     * Called when the OpMode is ready to process tasks.
     * This will happen when the user has selected an OpMode, or if setOpModes() returned null,
     * in which case it will run immediately after onInitialisation() has completed.
     * This is where you should add your tasks to the run queue.
     *
     * @param selectedOpMode the OpMode selected by the user, if applicable. Will be "DEFAULT" if no OpModeSelections were defined, or
     *                       NULL if the user did not select an OpMode.
     * @see #addTask(AutoTask)
     */
    protected abstract void onQueueReady(@Nullable OpModeSelection selectedOpMode);

    /**
     * Override to this method to add extra code to the activeLoop.
     */
    protected void onActiveLoop() {
    }
}
