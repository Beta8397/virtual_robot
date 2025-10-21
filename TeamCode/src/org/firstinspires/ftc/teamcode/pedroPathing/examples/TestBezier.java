package org.firstinspires.ftc.teamcode.pedroPathing.examples;


import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(group="Pedro")
public class TestBezier extends OpMode {

    private Follower follower;

    private PathChain forwards;

    private PathChain backwards;

    boolean forward = true;

    /**
     * This initializes the Follower and creates the PathChain for the "circle". Additionally, this
     * initializes the FTC Dashboard telemetry.
     */
    @Override
    public void init() {

        follower = Constants.createFollower(hardwareMap);

        follower.setStartingPose(new Pose(48, 48, Math.toRadians(90)));

        forwards = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(48.000, 48.000),
                        new Pose(47.241, 79.399),
                        new Pose(96.759, 65.170),
                        new Pose(96.000, 96.000)
                ))
                .setTangentHeadingInterpolation()
                .build();

        backwards = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(96.000, 96.000),
                        new Pose(95.336, 66.877),
                        new Pose(47.241, 78.545),
                        new Pose(48.000, 48.000)
                ))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();
    }

    @Override
    public void start(){
        follower.followPath(forwards);
    }

    /**
     * This runs the OpMode, updating the Follower as well as printing out the debug statements to
     * the Telemetry, as well as the FTC Dashboard.
     */
    @Override
    public void loop() {
        follower.update();
        if (!follower.isBusy()) {
            if (forward) {
                forward = false;
                follower.followPath(backwards);
            } else {
                forward = true;
                follower.followPath(forwards);
            }
        }
    }
}