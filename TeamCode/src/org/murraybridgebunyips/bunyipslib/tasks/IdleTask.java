package org.murraybridgebunyips.bunyipslib.tasks;

import org.murraybridgebunyips.bunyipslib.tasks.bases.ForeverTask;

/**
 * Task that runs forever but does nothing.
 * This is used as a default task on subsystems that don't have a default task.
 */
public class IdleTask extends ForeverTask {
    @Override
    public void init() {
        // no-op
    }

    @Override
    public void periodic() {
        // no-op
    }

    @Override
    public void onFinish() {
        // no-op
    }
}
