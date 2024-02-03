package org.murraybridgebunyips.bunyipslib.tasks.bases;

import org.murraybridgebunyips.bunyipslib.BunyipsSubsystem;

/**
 * A task that runs once and then immediately completes.
 */
public abstract class RunOnceTask extends Task {
    protected RunOnceTask() {
        super(0.0);
    }

    protected RunOnceTask(BunyipsSubsystem dependencySubsystem, boolean shouldOverrideConflictingTasks) {
        super(0.0, dependencySubsystem, shouldOverrideConflictingTasks);
    }

    @Override
    public final void init() {
        runOnce();
    }

    @Override
    public final void periodic() {
    }

    @Override
    public final boolean isTaskFinished() {
        return true;
    }

    @Override
    public final void onFinish() {
    }

    public abstract void runOnce();
}
