package org.firstinspires.ftc.teamcode.glados.tasks;

import androidx.annotation.NonNull;
import org.firstinspires.ftc.teamcode.common.BunyipsOpMode;
import org.firstinspires.ftc.teamcode.common.tasks.Task;
import org.firstinspires.ftc.teamcode.glados.components.GLaDOSArmCore;

public class GLaDOSMoveArmTask extends Task {
    private final GLaDOSArmCore arm;
    private final int sliderPosition;
    private final double rotatorDegrees;

    public GLaDOSMoveArmTask(@NonNull BunyipsOpMode opMode, double time, GLaDOSArmCore arm, int sliderPosition, double rotatorDegrees) {
        super(opMode, time);
        this.arm = arm;
        this.sliderPosition = sliderPosition;
        this.rotatorDegrees = rotatorDegrees;
    }

    @Override
    public void init() {
        arm.getSliderController().setTargetAngle(rotatorDegrees);
        arm.getSliderController().setExtrusionLength(sliderPosition, getTime());
    }

    @Override
    public void run() {
        arm.update();
    }

    @Override
    public boolean isTaskFinished() {
        return false;
    }

    @Override
    public void onFinish() {
        // noop
    }
}
