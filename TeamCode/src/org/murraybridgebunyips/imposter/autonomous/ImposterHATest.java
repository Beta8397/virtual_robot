package org.murraybridgebunyips.imposter.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;
import org.murraybridgebunyips.bunyipslib.AutonomousBunyipsOpMode;
import org.murraybridgebunyips.bunyipslib.Controls;
import org.murraybridgebunyips.bunyipslib.Reference;
import org.murraybridgebunyips.bunyipslib.RoadRunner;
import org.murraybridgebunyips.bunyipslib.roadrunner.drive.RoadRunnerDrive;
import org.murraybridgebunyips.bunyipslib.subsystems.HoldableActuator;
import org.murraybridgebunyips.imposter.components.ImposterConfig;

@Autonomous(name = "ImposterHATest", group = "VIRTUAL_BUNYIPSFTC")
public class ImposterHATest extends AutonomousBunyipsOpMode {
    private final ImposterConfig robot = new ImposterConfig();
    private HoldableActuator arm;

    @Override
    protected void onInitialise() {
        robot.init();
        arm = new HoldableActuator(robot.back_left_motor);
        onActiveLoop(() -> telemetry.add("arm power: %", robot.back_left_motor.getPower()));
    }

    @Override
    protected void onReady(@Nullable Reference<?> selectedOpMode, Controls selectedButton) {
        for (int i = 0; i < 10; i++) {
            addTask(arm.deltaTask(10000));
            addTask(arm.deltaTask(-10000));
        }
    }
}
