package org.firstinspires.ftc.teamcode.pedropathing.tuning.tests;


import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Pose;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.pedropathing.tuning.Constants;

@TeleOp(group = "PedroTests")
public class TestsLocalization extends LinearOpMode {

    Localizer localizer;
    Drivetrain drivetrain;

    @Override
    public void runOpMode() throws InterruptedException {
        localizer = new PinpointLocalizer(hardwareMap, Constants.localizerConfig);
        drivetrain = new Mecanum(hardwareMap, Constants.drivetrainConfig);

        localizer.setPose(Pose.zero());

        Thread.sleep(1000);
        waitForStart();
        localizer.setPose(Pose.zero());

        while (opModeIsActive()) {
            drivetrain.drive(new DrivePowers(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x), true);
            localizer.update();
            Pose pose = localizer.pose();
            telemetry.addData("Pose", "x: %.1f  y: %.1f  h: %.1f",
                    pose.x(), pose.y(), Math.toDegrees(pose.heading()));
            telemetry.update();
        }
    }
}

