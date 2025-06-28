package pedroPathing.examples;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

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
 */
@Config
@Autonomous (name = "Test Bezier", group = "Pedro")
public class TestBezier extends OpMode {
    private Telemetry telemetryA;

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

        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);

        follower.setStartingPose(new Pose(48, 48, Math.toRadians(90)));

        forwards = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(48.000, 48.000, Point.CARTESIAN),
                        new Point(47.241, 79.399, Point.CARTESIAN),
                        new Point(96.759, 65.170, Point.CARTESIAN),
                        new Point(96.000, 96.000, Point.CARTESIAN)
                ))
                .setTangentHeadingInterpolation()
                .build();

        backwards = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(96.000, 96.000, Point.CARTESIAN),
                        new Point(95.336, 66.877, Point.CARTESIAN),
                        new Point(47.241, 78.545, Point.CARTESIAN),
                        new Point(48.000, 48.000, Point.CARTESIAN)
                ))
                .setTangentHeadingInterpolation()
                .setReversed(true)
                .build();

        follower.followPath(forwards, true);

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
