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

    private Threads() {
    }

    /**
     * Start a new thread with the given task.
     *
     * @param task the runnable task to run on the new thread
     */
    public static void start(Runnable task) {
        Dbg.logd(Threads.class, "starting new thread: % ...", task.getClass().getSimpleName());
        Thread thread = new Thread(task);
        thread.setName(task.getClass().getSimpleName());
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
        for (Thread thread : threads.values()) {
            Dbg.logd(Threads.class, "stopping thread: % ...", thread.getName());
            thread.interrupt();
        }
        threads.clear();
    }

    /**
     * Check if a task is currently running.
     *
     * @param task the task to check, must be managed by Threads
     * @return true if the task is running, false otherwise
     */
    public static boolean isRunning(Runnable task) {
        Thread thread = threads.get(task.hashCode());
        return thread != null && thread.isAlive();
    }

    /**
     * Stop a specific task that is currently running.
     *
     * @param task the task to stop, must be managed by Threads
     */
    public static void stop(Runnable task) {
        Thread thread = threads.get(task.hashCode());
        if (thread != null) {
            Dbg.logd(Threads.class, "stopping thread: % ...", thread.getName());
            thread.interrupt();
            threads.remove(task.hashCode());
        } else {
            Dbg.warn(Threads.class, "tried to stop a task '%' that is not being managed by Threads.", task.getClass().getSimpleName());
        }
    }

    /**
     * Restart a specific task by stopping it and then starting it again.
     *
     * @param task the task to restart, must be managed by Threads
     */
    public static void restart(Runnable task) {
        Dbg.logd(Threads.class, "attempting to restart task: % ...", task.getClass().getSimpleName());
        stop(task);
        start(task);
    }

    /**
     * Wait for a specific task to finish running.
     *
     * @param task the task to wait for, must be managed by Threads
     */
    public static void waitFor(Runnable task) {
        waitFor(task, false);
    }

    /**
     * Wait for a specific task to finish running, with the option to interrupt it.
     *
     * @param task      the task to wait for, must be managed by Threads
     * @param interrupt whether to interrupt the task first then wait
     */
    public static void waitFor(Runnable task, boolean interrupt) {
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
