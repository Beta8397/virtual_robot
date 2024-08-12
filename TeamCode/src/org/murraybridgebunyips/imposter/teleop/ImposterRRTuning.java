package org.murraybridgebunyips.imposter.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.jetbrains.annotations.NotNull;
import org.murraybridgebunyips.bunyipslib.roadrunner.drive.MecanumRoadRunnerDrive;
import org.murraybridgebunyips.bunyipslib.roadrunner.drive.RoadRunnerDrive;
import org.murraybridgebunyips.bunyipslib.roadrunner.drive.localizers.ThreeWheelLocalizer;
import org.murraybridgebunyips.bunyipslib.roadrunner.drive.tuning.RoadRunnerTuning;
import org.murraybridgebunyips.imposter.components.ImposterConfig;

import java.util.ArrayList;

@TeleOp
public class ImposterRRTuning extends RoadRunnerTuning {
    @NotNull
    @Override
    protected RoadRunnerDrive getBaseRoadRunnerDrive() {
        ImposterConfig c = new ImposterConfig();
        c.init(this);
        RoadRunnerDrive drive = new MecanumRoadRunnerDrive(null, c.driveConstants, c.mecanumCoefficients, hardwareMap.voltageSensor, null, c.front_left_motor, c.front_right_motor, c.back_left_motor, c.back_right_motor);
        drive.setLocalizer(new ThreeWheelLocalizer(c.localizerCoefficients, c.enc_left, c.enc_right, c.enc_x, new ArrayList<>(), new ArrayList<>()));
        return drive;
    }
}
