package org.murraybridgebunyips.bunyipslib.tasks;

import org.murraybridgebunyips.bunyipslib.tasks.bases.ForeverTask;

/**
 * Task that runs forever but does nothing.
 * This is used as a default task on subsystems that don't have a default task.
 * IdleTask is ignored from subsystem report telemetry via a string match to "IdleTask".
 */
public class IdleTask extends ForeverTask {
    @Override
    protected void init() {
        // no-op
    }

    @Override
    protected void periodic() {
        // no-op
    }

    @Override
    protected void onFinish() {
        // no-op
    }
}
