package virtual_robot.controller;

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
                try {
                    this.wait();
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                    return;
                }
        }
    }


    /**
     * Puts the current thread to sleep for a bit as it has nothing better to do. This allows other
     * threads in the system to run.
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
     * @return whether the OpMode is currently active. If this returns false, you should
     * break out of the loop in your runOpMode() method and return to its caller.
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
     */
    public final boolean isStarted() {
        return this.isStarted || Thread.currentThread().isInterrupted();
    }

    /**
     * Has the the stopping of the opMode been requested?
     *
     * @return whether stopping opMode has been requested or not
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
     * This signals the runOpMode method that it should exit, by setting stopRequested to true. Then,
     * it attempts to shut down the executorService.
     */
    @Override
    final public void stop() {

        // make isStopRequested() return true (and opModeIsActive() return false)
        stopRequested = true;

        //This method may run twice; no need to shut down executorService if it's already terminated
        if (executorService != null && !executorService.isTerminated()) {  // paranoia

            // interrupt the linear opMode and shutdown it's service thread
            executorService.shutdownNow();

            try {
                executorService.awaitTermination(1000, TimeUnit.MILLISECONDS);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }

            if (!executorService.isTerminated()){
                System.out.println("Termination of executor service for runOpMode has timed out or been interrupted.");
                System.out.println("Do all loops in the runOpMode method check opModeIsActive()?");
            } else {
                System.out.println("Executor service for runOpMode has terminated successfully.");
            }
        }
    }

    protected void handleLoop() {

        //If runOpMode has exited, check for exceptions, shut down the executorService, then interrupt the opMode thread (currentThread)
        if (helper.isFinished()) {
            if (helper.hasException()){
                System.out.println("Exception from runOpMode:");
                System.out.println(helper.getException().getClass().getName());
                System.out.println(helper.getException().getLocalizedMessage());
            }
            stop();
            Thread.currentThread().interrupt();
        }

        synchronized (this) {
            this.notifyAll();
        }

        System.out.println("handleLoop");

    }

    protected class LinearOpModeHelper implements Runnable {

        protected Exception exception = null;
        protected boolean isFinished = false;

        public LinearOpModeHelper() {
        }

        @Override
        public void run() {
            executorService.execute(new Runnable() {
                @Override
                public void run() {
                    exception = null;
                    isFinished = false;

                    try {
                        LinearOpMode.this.runOpMode();
                    } catch (Exception e) {
                        exception = e;
                    } finally {
                        // Do the necessary bookkeeping
                        isFinished = true;
                    }
                }
            });
        }

        public boolean hasException() {
            return (exception != null);
        }

        public Exception getException() {
            return exception;
        }

        public boolean isFinished() {
            return isFinished;
        }
    }

}
