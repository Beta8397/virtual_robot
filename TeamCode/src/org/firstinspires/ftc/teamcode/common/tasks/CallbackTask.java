package org.firstinspires.ftc.teamcode.common.tasks;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.teamcode.common.BunyipsOpMode;

public class CallbackTask extends RunOnceTask {
    private final Runnable callback;

    public CallbackTask(@NonNull BunyipsOpMode opMode, Runnable callback) {
        super(opMode);
        this.callback = callback;
    }

    @Override
    public void runOnce() {
        callback.run();
    }
}
