package org.murraybridgebunyips.bunyipslib;

import java.util.HashMap;

/**
 * Async utilities for running user code on different threads while having control/logging over them.
 *
 * @author Lucas Bubner, 2024
 * @see While
 */
public class Threads {
    private static final HashMap<Integer, Thread> threads = new HashMap<>();

    /**
     * Start a new thread with the given task.
     *
     * @param task the task to run on the new thread, must implement Runnable
     * @param <T>  the type of the task
     */
    public static <T extends Runnable> void start(T task) {
        Dbg.logd(Threads.class, "running new thread: % ...", task.getClass().getSimpleName());
        Thread thread = new Thread(task);
        thread.start();
        threads.put(task.hashCode(), thread);
    }

    /**
     * Stop all threads that are currently running.
     * This will interrupt all threads and remove them from the list of managed threads,
     * not including threads that were started outside of this class.
     * This method is automatically called at the end of a BunyipsOpMode.
     */
    public static void stopAll() {
        Dbg.logd(Threads.class, "stopping all threads...");
        for (Thread thread : threads.values()) {
            thread.interrupt();
        }
        threads.clear();
    }

    /**
     * Check if a task is currently running.
     *
     * @param task the task to check
     * @param <T>  the type of the task
     * @return true if the task is running, false otherwise
     */
    public static <T extends Runnable> boolean isRunning(T task) {
        Thread thread = threads.get(task.hashCode());
        return thread != null && thread.isAlive();
    }

    /**
     * Stop a specific task that is currently running.
     *
     * @param task the task to stop
     * @param <T>  the type of the task
     */
    public static <T extends Runnable> void stop(T task) {
        Thread thread = threads.get(task.hashCode());
        if (thread != null) {
            Dbg.logd(Threads.class, "stopping thread: % ...", task.getClass().getSimpleName());
            thread.interrupt();
            threads.remove(task.hashCode());
        } else {
            Dbg.warn(Threads.class, "tried to stop a task '%' that is not being managed by Threads.", task.getClass().getSimpleName());
        }
    }

    /**
     * Restart a specific task by stopping it and then starting it again.
     *
     * @param task the task to restart
     * @param <T>  the type of the task
     */
    public static <T extends Runnable> void restart(T task) {
        stop(task);
        start(task);
    }

    /**
     * Wait for a specific task to finish running.
     *
     * @param task the task to wait for
     * @param <T>  the type of the task
     */
    public static <T extends Runnable> void waitFor(T task) {
        waitFor(task, false);
    }

    /**
     * Wait for a specific task to finish running, with the option to interrupt it.
     *
     * @param task      the task to wait for
     * @param interrupt whether to interrupt the task first then wait
     * @param <T>       the type of the task
     */
    public static <T extends Runnable> void waitFor(T task, boolean interrupt) {
        Thread thread = threads.get(task.hashCode());
        if (thread != null) {
            if (interrupt) {
                Dbg.logd(Threads.class, "stopping thread: % ...", task.getClass().getSimpleName());
                thread.interrupt();
            }
            try {
                Dbg.logd(Threads.class, "waiting for thread: % ...", task.getClass().getSimpleName());
                thread.join();
            } catch (InterruptedException e) {
                Dbg.error(Threads.class, "thread '%' was interrupted while waiting.", task.getClass().getSimpleName());
            }
        } else {
            Dbg.warn(Threads.class, "tried to wait for a task '%' that is not being managed by Threads.", task.getClass().getSimpleName());
        }
    }
}
