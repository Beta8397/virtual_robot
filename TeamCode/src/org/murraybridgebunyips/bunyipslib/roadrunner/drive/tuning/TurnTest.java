package org.murraybridgebunyips.bunyipslib.roadrunner.drive.tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.murraybridgebunyips.bunyipslib.roadrunner.drive.RoadRunnerDrive;


/*
 * This is a simple routine to test turning capabilities.
 */
//@Config
public abstract class TurnTest extends LinearOpMode {
    public static double ANGLE = 90; // deg
    protected RoadRunnerDrive drive;

    @Override
    public void runOpMode() {
        if (drive == null) throw new NullPointerException("drive is null!");

        waitForStart();

        if (isStopRequested()) return;

        drive.turn(Math.toRadians(ANGLE));
    }
}