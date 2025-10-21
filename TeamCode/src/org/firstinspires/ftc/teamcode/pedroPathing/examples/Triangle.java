package org.firstinspires.ftc.teamcode.pedroPathing.examples;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/**
 * This is the Triangle autonomous OpMode.
 * It runs the robot in a triangle, with the starting point being the bottom-middle point.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @author Samarth Mahapatra - 1002 CircuitRunners Robotics Surge
 * @version 1.0, 12/30/2024
 *
 * Adapted by J. Kenney for use in virtual_robot.
 */
@Autonomous(group="Pedro")
public class Triangle extends OpMode {

    Follower follower;

    private final Pose startPose = new Pose(0, 0, Math.toRadians(0));
    private final Pose interPose = new Pose(24, -24, Math.toRadians(90));
    private final Pose endPose = new Pose(24, 24, Math.toRadians(45));

    private PathChain triangle;

    /**
     * This runs the OpMode, updating the Follower as well as printing out the debug statements to
     * the Telemetry, as well as the Panels.
     */
    @Override
    public void loop() {
        follower.update();

        if (follower.atParametricEnd()) {
            follower.followPath(triangle, true);
        }
    }

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
    }

    @Override
    public void init_loop() {
        telemetry.addData("Press START when ready.","");
        follower.update();
    }

    /** Creates the PathChain for the "triangle".*/
    @Override
    public void start() {
        follower.setStartingPose(startPose);

        triangle = follower.pathBuilder()
                .addPath(new BezierLine(startPose, interPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), interPose.getHeading())
                .addPath(new BezierLine(interPose, endPose))
                .setLinearHeadingInterpolation(interPose.getHeading(), endPose.getHeading())
                .addPath(new BezierLine(endPose, startPose))
                .setLinearHeadingInterpolation(endPose.getHeading(), startPose.getHeading())
                .build();

        follower.followPath(triangle);
    }
}