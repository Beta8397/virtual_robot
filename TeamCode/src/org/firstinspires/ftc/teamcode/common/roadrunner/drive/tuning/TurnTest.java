package org.firstinspires.ftc.teamcode.common.roadrunner.drive.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.roadrunner.drive.MecanumRoadRunnerDrive;


/*
 * This is a simple routine to test turning capabilities.
 */
@Config
public abstract class TurnTest extends LinearOpMode {
    public static double ANGLE = 90; // deg
    protected MecanumRoadRunnerDrive drive;

    @Override
    public void runOpMode() {
        if (drive == null) throw new NullPointerException("drive is null!");

        waitForStart();

        if (isStopRequested()) return;

        drive.turn(Math.toRadians(ANGLE));
    }
}