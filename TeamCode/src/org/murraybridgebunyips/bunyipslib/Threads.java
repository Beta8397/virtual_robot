package org.murraybridgebunyips.bunyipslib;

import java.util.HashMap;

/**
 * Async utilities for running user code on different threads while having control/logging over them.
 *
 * @author Lucas Bubner, 2024
 * @see While
 */
public final class Threads {
    private static final HashMap<Integer, Thread> threads = new HashMap<>();

    private Threads() {
    }

    /**
     * Start a new thread with the given task.
     *
     * @param task the runnable task to run on the new thread
     * @param name the name of the thread to access it later and to log as
     */
    public static void start(Runnable task, String name) {
        Dbg.logd(Threads.class, "starting new thread: % ...", name);
        Thread thread = new Thread(task);
        thread.setName(name);
        thread.start();
        threads.put(task.hashCode(), thread);
    }

    /**
     * Start a new thread with the given infinite loop task.
     * This thread will auto end when the task is interrupted.
     *
     * @param task the infinite loop task to run on the new thread
     * @param name the name of the thread to access it later and to log as
     */
    public static void startLoop(Runnable task, String name) {
        Dbg.logd(Threads.class, "starting new loop thread: % ...", name);
        Thread thread = new Thread(() -> {
            while (!Thread.currentThread().isInterrupted()) {
                task.run();
            }
        });
        thread.setName(name);
        thread.start();
        threads.put(task.hashCode(), thread);
    }

    /**
     * Start a new thread with the given task.
     *
     * @param task the runnable task to run on the new thread
     */
    public static void start(Runnable task) {
        start(task, task.getClass().getSimpleName());
    }

    /**
     * Start a new thread with the given infinite loop task.
     * This thread will auto end when the task is interrupted.
     *
     * @param task the infinite loop task to run on the new thread
     */
    public static void startLoop(Runnable task) {
        startLoop(task, task.getClass().getSimpleName());
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
     * Check if a task is currently running.
     *
     * @param task the name of the task to check, must be managed by Threads
     * @return true if the task is running, false otherwise
     */
    public static boolean isRunning(String task) {
        for (Thread thread : threads.values()) {
            if (thread.getName().equals(task)) {
                return thread.isAlive();
            }
        }
        return false;
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
     * Stop a specific task that is currently running.
     *
     * @param task the name of the task to stop, must be managed by Threads
     */
    public static void stop(String task) {
        for (Thread thread : threads.values()) {
            if (thread.getName().equals(task)) {
                Dbg.logd(Threads.class, "stopping thread: % ...", thread.getName());
                thread.interrupt();
                threads.remove(thread.hashCode());
                return;
            }
        }
        Dbg.warn(Threads.class, "tried to stop a task '%' that is not being managed by Threads.", task);
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
     * Restart a specific task by stopping it and then starting it again.
     *
     * @param task the name of the task to restart, must be managed by Threads
     */
    public static void restart(String task) {
        Dbg.logd(Threads.class, "attempting to restart task: % ...", task);
        for (Thread thread : threads.values()) {
            if (thread.getName().equals(task)) {
                stop(thread.getName());
                start(thread, thread.getName());
                return;
            }
        }
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
     * Wait for a specific task to finish running.
     *
     * @param task the name of the task to wait for, must be managed by Threads
     */
    public static void waitFor(String task) {
        waitFor(task, false);
    }

    /**
     * Wait for a specific task to finish running, with the option to interrupt it.
     *
     * @param task      the name of the task to wait for, must be managed by Threads
     * @param interrupt whether to interrupt the task first then wait
     */
    public static void waitFor(String task, boolean interrupt) {
        for (Thread thread : threads.values()) {
            if (thread.getName().equals(task)) {
                waitFor(thread, interrupt);
                return;
            }
        }
        Dbg.warn(Threads.class, "tried to wait for a task '%' that is not being managed by Threads.", task);
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
