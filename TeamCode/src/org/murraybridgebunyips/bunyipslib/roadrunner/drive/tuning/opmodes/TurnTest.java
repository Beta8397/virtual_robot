package org.murraybridgebunyips.bunyipslib.roadrunner.drive.tuning.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.murraybridgebunyips.bunyipslib.DualTelemetry;
import org.murraybridgebunyips.bunyipslib.TriConsumer;
import org.murraybridgebunyips.bunyipslib.roadrunner.drive.RoadRunnerDrive;

/**
 * This is a simple routine to test turning capabilities.
 */
public class TurnTest implements TriConsumer<LinearOpMode, DualTelemetry, RoadRunnerDrive> {
    /**
     * The angle to turn for the turn test routine.
     */
    public double ANGLE_DEGREES = 90;

    @Override
    public void accept(LinearOpMode opMode, DualTelemetry telemetry, RoadRunnerDrive drive) {
        opMode.waitForStart();
        if (opMode.isStopRequested()) return;
        drive.turn(Math.toRadians(ANGLE_DEGREES));
    }
}