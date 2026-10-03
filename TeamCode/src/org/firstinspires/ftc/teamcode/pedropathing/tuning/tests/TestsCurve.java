package org.firstinspires.ftc.teamcode.pedropathing.tuning.tests;


import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerLog;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.pedropathing.tuning.Constants;

import java.util.function.Function;

import static com.pedropathing.api.Paths.curve;

@Autonomous(group = "PedroTests")
public class TestsCurve extends LinearOpMode {
    double distance = 48;
    Follower follower;
    FollowerLog logger;


    @Override
    public void runOpMode() throws InterruptedException {
        Follower follower = Constants.create(hardwareMap);
        follower.setPose(Pose.zero());

        double distance = 48;
        boolean forward = true;

        Path path1 = curve(Pose.zero(), new Pose(distance + 0,0), new Pose(distance,distance)).tangent();
        Path path2 = curve(new Pose(distance,distance), new Pose(distance,0), Pose.zero()).tangent();

        Thread.sleep(1000);
        waitForStart();
        follower.setPose(Pose.zero());
        follower.update();
        follower.follow(path1);

        while (opModeIsActive() && !gamepad1.a) {
            follower.update();
            Pose pose = follower.pose();
            telemetry.addData("Pose", "x: %.1f  y: %.1f  h: %.1f",
                    pose.x(), pose.y(), Math.toDegrees(pose.heading()));
            telemetry.update();
            if (follower.atParametricEnd()) {
                if (forward) {
                    follower.follow(path2);
                } else {
                    follower.follow(path1);
                }
                forward = !forward;
            }
        }

        follower.drivetrain.stop(true);

        while (opModeIsActive()){
            continue;
        }
    }
}
