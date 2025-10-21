package org.firstinspires.ftc.teamcode.pedroPathing.examples;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/**
 * This is the Circle autonomous OpMode. It runs the robot in a PathChain that's actually not quite
 * a circle, but some Bezier curves that have control points set essentially in a square. However,
 * it turns enough to tune your centripetal force correction and some of your heading. Some lag in
 * heading is to be expected.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @version 1.0, 3/12/2024
 *
 * Adapted by J. Kenney for use in virtual_robot.
 */
@Autonomous(group="Pedro")
public class Circle extends OpMode {

    Follower follower;
    public static double RADIUS = 10;
    private PathChain circle;

    public void start() {
        circle = follower.pathBuilder()
                .addPath(new BezierCurve(new Pose(0, 0), new Pose(RADIUS, 0), new Pose(RADIUS, RADIUS)))
                .setHeadingInterpolation(HeadingInterpolator.facingPoint(0, RADIUS))
                .addPath(new BezierCurve(new Pose(RADIUS, RADIUS), new Pose(RADIUS, 2 * RADIUS), new Pose(0, 2 * RADIUS)))
                .setHeadingInterpolation(HeadingInterpolator.facingPoint(0, RADIUS))
                .addPath(new BezierCurve(new Pose(0, 2 * RADIUS), new Pose(-RADIUS, 2 * RADIUS), new Pose(-RADIUS, RADIUS)))
                .setHeadingInterpolation(HeadingInterpolator.facingPoint(0, RADIUS))
                .addPath(new BezierCurve(new Pose(-RADIUS, RADIUS), new Pose(-RADIUS, 0), new Pose(0, 0)))
                .setHeadingInterpolation(HeadingInterpolator.facingPoint(0, RADIUS))
                .build();
        follower.followPath(circle);
    }

    @Override
    public void init_loop() {
        telemetry.addData("Press START when ready.","");
        follower.update();
    }

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
    }

    /**
     * This runs the OpMode, updating the Follower as well as printing out the debug statements to
     * the Telemetry, as well as the FTC Dashboard.
     */
    @Override
    public void loop() {
        follower.update();

        if (follower.atParametricEnd()) {
            follower.followPath(circle);
        }
    }
}