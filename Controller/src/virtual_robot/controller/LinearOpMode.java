package virtual_robot.controller;

import virtual_robot.controller.OpMode;

import java.util.concurrent.CancellationException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;

/**
 * This is copied mostly from the FTC SDK 4.3
 */
public abstract class LinearOpMode extends OpMode {
    private volatile boolean isStarted = false;
    private volatile boolean stopRequested = false;
    private LinearOpModeHelper helper = null;
    private ExecutorService executorService = null;

    /**
     * OpModes must override the abstract runOpMode() method.
     */
    abstract public void runOpMode() throws InterruptedException;


    /**
     * Pauses the Linear Op Mode until start has been pressed or until the current thread
     * is interrupted.
     */
    public synchronized void waitForStart() {
        while (!isStarted()) {
            synchronized (this) {
                try {
                    this.wait();
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                    return;
                }
            }
        }
    }


    /**
     * Puts the current thread to sleep for a bit as it has nothing better to do. This allows other
     * threads in the system to run.
     *
     * <p>One can use this method when you have nothing better to do in your code as you await state
     * managed by other threads to change. Calling idle() is entirely optional: it just helps make
     * the system a little more responsive and a little more efficient.</p>
     *
     * <p>{@link #idle()} is conceptually related to waitOneFullHardwareCycle(), but makes no
     * guarantees as to completing any particular number of hardware cycles, if any.</p>
     *
     * @see #opModeIsActive()
     */
    public final void idle() {
        // Otherwise, yield back our thread scheduling quantum and give other threads at
        // our priority level a chance to run
        Thread.yield();
    }


    /**
     * Pause execution of the OpMode for the indicated number of milliseconds
     * @param milliseconds
     *
     * Note: during this pause, the motors will continue running at their previous power settings.
     */
    protected void sleep(long milliseconds){
        if (Thread.currentThread().isInterrupted()) return;
        try{
            Thread.sleep(milliseconds);
        } catch(InterruptedException exc){
            Thread.currentThread().interrupt();
        }
        return;
    }

    /**
     * Answer as to whether this opMode is active and the robot should continue onwards. If the
     * opMode is not active, the OpMode should terminate at its earliest convenience.
     *
     * <p>Note that internally this method calls {@link #idle()}</p>
     *
     * @return whether the OpMode is currently active. If this returns false, you should
     * break out of the loop in your {@link #runOpMode()} method and return to its caller.
     * @see #runOpMode()
     * @see #isStarted()
     * @see #isStopRequested()
     */
    public final boolean opModeIsActive() {
        boolean isActive = !this.isStopRequested() && this.isStarted();
        if (isActive) {
            idle();
        }
        return isActive;
    }

    /**
     * Has the opMode been started?
     *
     * @return whether this opMode has been started or not
     * @see #opModeIsActive()
     * @see #isStopRequested()
     */
    public final boolean isStarted() {
        return this.isStarted || Thread.currentThread().isInterrupted();
    }

    /**
     * Has the the stopping of the opMode been requested?
     *
     * @return whether stopping opMode has been requested or not
     * @see #opModeIsActive()
     * @see #isStarted()
     */
    public final boolean isStopRequested() {
        return this.stopRequested || Thread.currentThread().isInterrupted();
    }

    /**
     * From the non-linear OpMode; do not override
     */
    @Override
    final public void init() {
        this.executorService = Executors.newSingleThreadExecutor();
        this.helper = new LinearOpModeHelper();
        this.isStarted = false;
        this.stopRequested = false;

        this.executorService.execute(helper);
    }

    /**
     * From the non-linear OpMode; do not override
     */
    @Override
    final public void init_loop() {
        handleLoop();
    }

    /**
     * From the non-linear OpMode; do not override
     */
    @Override
    final public void start() {
        stopRequested = false;
        isStarted = true;
        synchronized (this) {
            this.notifyAll();
        }
    }

    /**
     * From the non-linear OpMode; do not override
     */
    @Override
    final public void loop() {
        handleLoop();
    }

    /**
     * From the non-linear OpMode; do not override
     */
    @Override
    final public void stop() {

        // make isStopRequested() return true (and opModeIsActive() return false)
        stopRequested = true;

        if (executorService != null) {  // paranoia

            // interrupt the linear opMode and shutdown it's service thread
            executorService.shutdownNow();

            /** Wait, forever, for the OpMode to stop. If this takes too long, then
             * {@link OpModeManagerImpl#callActiveOpModeStop()} will catch that and take action */
            try {
                String serviceName = "user linear op mode";
                executorService.awaitTermination(100, TimeUnit.DAYS);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }
    }

    protected void handleLoop() {
        // if there is a runtime exception in user code; throw it so the normal error
        // reporting process can handle it
        if (helper.hasRuntimeException()) {
            throw helper.getRuntimeException();
        }

        synchronized (this) {
            this.notifyAll();
        }
    }

    protected class LinearOpModeHelper implements Runnable {

        protected RuntimeException exception = null;
        protected boolean isShutdown = false;

        public LinearOpModeHelper() {
        }

        @Override
        public void run() {
            executorService.execute(new Runnable() {
                @Override
                public void run() {
                    exception = null;
                    isShutdown = false;

                    try {
                        LinearOpMode.this.runOpMode();
                        requestOpModeStop();
                    } catch (InterruptedException ie) {
                        // InterruptedException, shutting down the op mode
                        System.out.println("LinearOpMode received an InterruptedException; shutting down this linear op mode");
                    } catch (CancellationException ie) {
                        // In our system, CancellationExceptions are thrown when data was trying to be acquired, but
                        // an interrupt occurred, and you're in the unfortunate situation that the data acquisition API
                        // involved doesn't allow InterruptedExceptions to be thrown. You can't return (what data would
                        // you return?), and so you have to throw a RuntimeException. CancellationException seems the
                        // best choice.
                        System.out.println("LinearOpMode received a CancellationException; shutting down this linear op mode");
                    } catch (RuntimeException e) {
                        exception = e;
                    } finally {
                        // Do the necessary bookkeeping
                        isShutdown = true;
                    }
                }
            });
        }

        public boolean hasRuntimeException() {
            return (exception != null);
        }

        public RuntimeException getRuntimeException() {
            return exception;
        }

        public boolean isShutdown() {
            return isShutdown;
        }
    }

}
