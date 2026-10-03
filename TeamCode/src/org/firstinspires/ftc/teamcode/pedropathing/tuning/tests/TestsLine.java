package org.firstinspires.ftc.teamcode.pedropathing.tuning.tests;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.pedropathing.tuning.Constants;
import static com.pedropathing.api.Paths.line;

@Autonomous(group = "PedroTests")
public class TestsLine extends LinearOpMode {
    double distance = 48;
    Follower follower;

    @Override
    public void runOpMode() throws InterruptedException {
        follower = Constants.create(hardwareMap);
        follower.setPose(new Pose(0,0,Math.toRadians(90)));

        double distance = 48;
        boolean forward = true;

        Path path1 = line(new Pose(0,0,0), new Pose(distance,0, 0)).tangent();
        Path path2 = line(new Pose(distance,0, 0), new Pose(0,0,0)).reverseTangent();

        Thread.sleep(1000);
        waitForStart();
        follower.setPose(new Pose(0,0,0));
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

