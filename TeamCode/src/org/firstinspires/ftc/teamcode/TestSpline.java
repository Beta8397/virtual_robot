package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Curve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.teamcode.pathutil.CubicSpline2D;
import org.firstinspires.ftc.teamcode.pathutil.Spline;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.ArrayList;
import java.util.Arrays;

@Autonomous(group = "Pedro")
public class TestSpline extends OpMode {

    PathChain forward, reverse;
    Follower follower;
    boolean fwd = true;

    public void init(){
        follower = Constants.createFollower(hardwareMap);

        forward = follower.pathBuilder()
                .addPath(new Spline(Math.toRadians(90), Math.toRadians(90),
                        Constants.pathConstraints, new Pose(-24,-24), new Pose(0,0),
                        new Pose(24,24))
                )
                .setTangentHeadingInterpolation()
                .build();

        reverse = follower.pathBuilder()
                .addPath(new Spline(Math.toRadians(-90), Math.toRadians(-90),
                        Constants.pathConstraints, new Pose(24,24), new Pose(0, 0),
                        new Pose(-24,-24)))
                .setTangentHeadingInterpolation().setReversed()
                .build();

    }

    public void start(){
        System.out.println("Start method called");
        follower.setPose(new Pose(-24, -24, Math.toRadians(90)));
        follower.followPath(forward);
    }

    public void loop(){
        follower.update();

        if (!follower.isBusy()){
            if (fwd){
                System.out.println("switching to reverse path");
                fwd = false;
                follower.followPath(reverse);
            } else {
                System.out.println("Switching to forward path");
                fwd = true;
                follower.followPath(forward);
            }
        }
    }

}
