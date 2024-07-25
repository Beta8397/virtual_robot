package org.murraybridgebunyips.imposter.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;
import org.murraybridgebunyips.bunyipslib.AutonomousBunyipsOpMode;
import org.murraybridgebunyips.bunyipslib.Controls;
import org.murraybridgebunyips.bunyipslib.Reference;
import org.murraybridgebunyips.bunyipslib.RoadRunner;
import org.murraybridgebunyips.bunyipslib.drive.MecanumDrive;
import org.murraybridgebunyips.bunyipslib.roadrunner.drive.RoadRunnerDrive;
import org.murraybridgebunyips.imposter.components.ImposterConfig;

@Autonomous
public class ImposterBatchRRTaskQA extends AutonomousBunyipsOpMode implements RoadRunner {
    private final ImposterConfig config = new ImposterConfig();
    private MecanumDrive drive;
    @Override
    protected void onInitialise() {
        config.init();
        drive = new MecanumDrive(config.driveConstants, config.mecanumCoefficients, hardwareMap.voltageSensor, config.imu, config.front_left_motor, config.front_right_motor, config.back_left_motor, config.back_right_motor);
    }

    @Override
    protected void onReady(@Nullable Reference<?> selectedOpMode, Controls selectedButton) {
        for (int i = 0; i < 100; i++) {
            makeTrajectory().forward(3).addTask();
            makeTrajectory().strafeRight(3).addTask();
            makeTrajectory().back(3).addTask();
            makeTrajectory().strafeLeft(3).addTask();
        }
    }

    @NotNull
    @Override
    public RoadRunnerDrive getDrive() {
        return drive;
    }
}
