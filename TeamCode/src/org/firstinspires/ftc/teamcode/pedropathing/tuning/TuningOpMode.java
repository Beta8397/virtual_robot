package org.firstinspires.ftc.teamcode.pedropathing.tuning;

import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.localization.Localizer;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(group = "PedroTune")
public abstract class TuningOpMode extends LinearOpMode {
    protected Localizer localizer;
    protected Drivetrain drivetrain;

    public void runOpMode() throws InterruptedException{
        System.out.println("runOpMode");
        waitForStart();
        localizer = new PinpointLocalizer(hardwareMap, (PinpointConfig)Constants.localizerConfig);
        drivetrain = new Mecanum(hardwareMap, (MecanumConfig)Constants.drivetrainConfig);
        runTuningOpMode();
    }

    public abstract void runTuningOpMode() throws  InterruptedException;

}
