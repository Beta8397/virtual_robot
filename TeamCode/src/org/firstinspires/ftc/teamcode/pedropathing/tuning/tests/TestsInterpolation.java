package org.firstinspires.ftc.teamcode.pedropathing.tuning.tests;


import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.interpolator.Interpolator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.pedropathing.tuning.Constants;

import static com.pedropathing.api.Paths.curve;

@Autonomous(group = "PedroTests")
public class TestsInterpolation extends LinearOpMode {
    double distance = 48;
    Follower follower;

    @Override
    public void runOpMode() throws InterruptedException {
        follower = Constants.create(hardwareMap);
        follower.setPose(Pose.zero());

        double distance = 48;
        boolean forward = true;

        Path path1 = curve(Pose.zero(), new Pose(distance + 0,0), new Pose(distance,distance)).heading((curve, t) -> Math.PI);
        Path path2 = curve(new Pose(distance,distance), new Pose(distance,0), Pose.zero()).heading(Interpolator.piecewise().until(0.5, Interpolator.tangent).until(1.0, Interpolator.constant(0)));

        Thread.sleep(1000);
        waitForStart();
        follower.setPose(Pose.zero());
        follower.update();
        follower.follow(path1);

        while (opModeIsActive()) {
            follower.update();
            if (follower.atParametricEnd()) {
                if (forward) {
                    follower.follow(path2);
                } else {
                    follower.follow(path1);
                }
                forward = !forward;
            }
        }
    }
}
