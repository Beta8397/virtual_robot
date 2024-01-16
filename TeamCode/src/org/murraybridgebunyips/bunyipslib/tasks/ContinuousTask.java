package org.murraybridgebunyips.bunyipslib.tasks;

import org.murraybridgebunyips.bunyipslib.tasks.bases.RunForeverTask;
import org.murraybridgebunyips.bunyipslib.tasks.bases.Task;

/**
 * A task to run continuously and will never finish.
 * <p>
 * {@code new ContinuousTask(() -> drive.update());}
 */
public class ContinuousTask extends RunForeverTask {
    private final Runnable callback;

    public ContinuousTask(Runnable callback) {
        this.callback = callback;
    }

    @Override
    public void run() {
        callback.run();
    }

    @Override
    public void init() {
        // noop
    }

    @Override
    public void onFinish() {
        // noop
    }
}
