package org.murraybridgebunyips.bunyipslib;

import androidx.annotation.NonNull;

import org.murraybridgebunyips.bunyipslib.tasks.RepeatTask;
import org.murraybridgebunyips.bunyipslib.tasks.bases.Task;

import java.util.ArrayList;
import java.util.Collections;

/**
 * Utility class for running {@link Task} instances from an environment that does not have a task scheduler.
 * This is useful for OpModes that are not using a command-based structure, such as the base {@link BunyipsOpMode},
 * but still needs support to run tasks on subsystems that only return Tasks. This class simplifies the task run process to
 * a more intermediate "register and run" system. Do note it is recommended to use a more integrated system such as
 * the {@link Scheduler}, which is integrated into {@link CommandBasedBunyipsOpMode}.
 * <p>
 * This class will allow you to simply pass a task instance that you wish to run (with run conditions being managed
 * at your own discretion), where continuous calls to {@code runRepeatedly()} will execute the task continually, resetting it if it is done.
 * This behaviour is similar to how the {@code run()} call works in the {@link Scheduler}, however, this comes with the guarantee
 * that a task will only execute any time it is being actively told to do so.
 * <p>
 * If you simply wish to run tasks with limited instance management, instantiating them directly in {@link #register}
 * and running them can be accomplished with an index, as done in the {@link #run}/{@link #runRepeatedly} overloads. This limits your ability
 * to manage/reset tasks, but is useful for simpler control loops.
 * <p>
 * This class is used maximally where you will manage the instances of the tasks you retrieve from your subsystems/methods, to allow you finer-grain
 * control over your tasks and their reset/runtime methods. Do note however it may be more worthwhile migrating to a command-based structure such
 * as the {@link Scheduler} or {@link CommandBasedBunyipsOpMode}.
 * This pattern is useful for the standard {@link #run} method that will execute one start-to-finish cycle of a task, with the added benefit
 * that a task will only iterate when being told to do so by methods in this utility.
 * <p>
 * Example usage:
 * <pre>
 * {@code
 *     // init-phase, index-based registration
 *     Tasks.register(arm.tasks.home(), claw.openTask(), ...);
 *     // active phase
 *     if (condition) {
 *       Tasks.run(0); // runs arm.tasks.home() one iteration, if it is finished it will no-op forever
 *       Tasks.runRepeatedly(1); // runs claw.openTask(), if it is finished it will be auto-reset and can run again
 *     }
 * }
 * </pre>
 * <pre>
 * {@code
 *     // init-phase, instance-based registration
 *     Task homeTask = arm.tasks.home();
 *     Task openTask = claw.openTask();
 *     Tasks.register(homeTask, openTask);
 *     // active phase
 *     if (condition) {
 *         Tasks.run(homeTask); // runs arm.tasks.home() one iteration, if it is finished it will no-op until a reset
 *         Tasks.run(1); // index-based from above still works, the only difference with this approach is that
 *                       // you have additional control over your tasks.
 *     }
 *     // reset can be accomplished as with any task, allowing run calls to work again
 *     if (... && homeTask.isFinished()) homeTask.reset();
 * }
 * </pre>
 *
 * @author Lucas Bubner, 2024
 * @see Scheduler
 * @see CommandBasedBunyipsOpMode
 */
public final class Tasks {
    private static final ArrayList<Task> registeredTasks = new ArrayList<>();

    private Tasks() {
    }

    static void resetForOpMode() {
        registeredTasks.clear();
    }

    /**
     * Register a series of tasks that can be executed at any time within the {@link #run} methods.
     * <p>
     * This is required as the common Task structure instantiates new tasks for every action, therefore making it
     * important to pre-register tasks you will use then referencing them in the future. This is automatically handled
     * in the current applications of Task, as these actions are constructed at init.
     * <p>
     * You may wish to pass in a class member or new instantiations to this method, depending on how you will access them
     * (either with an index, or using the instance itself as you may want to reset it)
     *
     * @param tasks tasks to register for use within the {@code run} methods.
     */
    public static void register(@NonNull Task... tasks) {
        Collections.addAll(registeredTasks, tasks);
    }

    /**
     * Run a single robot task iteration, where further calls will no-op if the task is finished.
     * <p>
     * This is for tasks that you may want to run start-to-finish only once,
     * where you don't want this task to auto-reset and execute after the task is done. Note that if this is the
     * only way for you to run this task (where you did not store a reference yourself), you do not have control over this task, making it so this task will run once
     * and never be able to requeue again (as the ability to reset it won't be possible due to a lack of handling the instance).
     * <p>
     * Resetting a task to run again can be accomplished on the task itself via {@code task.reset()}, where this method
     * will run another cycle of the task automatically on the next call.
     *
     * @param registeredIndex the existing index of the task as registered in order from {@link #register} parameters order.
     */
    public static void run(int registeredIndex) {
        Task task = registeredTasks.get(registeredIndex);
        if (task.isFinished()) {
            return;
        }
        task.run();
        task.pollFinished();
    }

    /**
     * Run a single robot task iteration, auto-resetting the task if it finishes.
     * This is the same as executing a task wrapped in a {@link RepeatTask}, useful for tasks that are "one and done".
     *
     * @param registeredIndex the existing index of the task as registered in order from {@link #register} parameters order.
     */
    public static void runRepeatedly(int registeredIndex) {
        Task task = registeredTasks.get(registeredIndex);
        if (task.isFinished()) {
            task.reset();
        }
        task.run();
        task.pollFinished();
    }

    /**
     * Run a single robot task iteration, where further calls will no-op if the task is finished.
     * This task must be registered prior through {@link #register}. Do not instantiate new tasks here.
     * <p>
     * This is for tasks that you may want to run start-to-finish only once,
     * where you don't want this task to auto-reset and execute after the task is done. You may choose to reset this
     * instance at your own discretion, using your own reference.
     * <p>
     * Resetting a task to run again can be accomplished on the task itself via {@code task.reset()}, where this method
     * will run another cycle of the task automatically on the next call.
     *
     * @param task the registered instance of the task that will be stored and subsequently run with additional calls to this method
     */
    public static void run(Task task) {
        run(registeredTasks.indexOf(task));
    }

    /**
     * Run a single robot task iteration, auto-resetting the task if it finishes.
     * This task must be registered prior through {@link #register}. Do not instantiate new tasks here.
     * This is the same as executing a task wrapped in a {@link RepeatTask}, useful for tasks that are "one and done".
     *
     * @param task the registered instance of the task that will be stored and subsequently run with additional calls to this method
     */
    public static void runRepeatedly(Task task) {
        runRepeatedly(registeredTasks.indexOf(task));
    }
}
