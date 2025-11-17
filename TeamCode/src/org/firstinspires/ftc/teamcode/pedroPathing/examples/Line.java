package org.firstinspires.ftc.teamcode.pedroPathing.examples;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.tuning.tuning_util.Adjuster;

import java.util.ArrayList;
import java.util.List;

/**
 * This is the Line Test Tuner OpMode. It will drive the robot forward and back
 * The user should push the robot laterally and angular to test out the drive, heading, and translational PIDFs.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @version 1.0, 3/12/2024
 *
 * Adapted by J. Kenney for use in virtual_robot.
 */
@Autonomous(group="Pedro")
public class Line extends OpMode {

    Follower follower;

    public static double DISTANCE = 48;
    private boolean forward = true;

    private Path forwards;
    private Path backwards;

    List<Adjuster> adjusters;
    Adjuster adjuster;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        adjusters = new ArrayList<>();

        adjusters.add(new Adjuster("P", ()->follower.getConstants().getCoefficientsDrivePIDF().P,
                (double p)->follower.getConstants().getCoefficientsDrivePIDF().P = p, gamepad1));
        adjusters.add(new Adjuster("D", ()->follower.getConstants().getCoefficientsDrivePIDF().D,
                (double d)->follower.getConstants().getCoefficientsDrivePIDF().D = d, gamepad1));
        adjusters.add(new Adjuster("F", ()->follower.getConstants().getCoefficientsDrivePIDF().F,
                (double f)->follower.getConstants().getCoefficientsDrivePIDF().F = f, gamepad1));
        adjusters.add(new Adjuster("Breaking Strength", ()->follower.getConstraints().getBrakingStrength(),
                (double bStrength)->follower.getConstraints().setBrakingStrength(bStrength), gamepad1));
        adjusters.add(new Adjuster("Braking Start", ()->follower.getConstraints().getBrakingStart(),
                (double bStart)->follower.getConstraints().setBrakingStart(bStart), gamepad1));
        adjuster = adjusters.get(0);

    }

    /** This initializes the Follower and creates the forward and backward Paths. */
    @Override
    public void init_loop() {
        telemetry.addData("Press START when ready.","");
        follower.update();
    }

    @Override
    public void start() {
        forwards = new Path(new BezierLine(new Pose(0,0), new Pose(DISTANCE,0)));
        forwards.setConstantHeadingInterpolation(0);
        backwards = new Path(new BezierLine(new Pose(DISTANCE,0), new Pose(0,0)));
        backwards.setConstantHeadingInterpolation(0);

        // follower.followPath(forwards);
        follower.followPath(forwards);
    }

    /** This runs the OpMode, updating the Follower as well as printing out the debug statements to the Telemetry */
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


        if (gamepad1.rightBumperWasPressed()){
            adjuster = adjusters.get((adjusters.indexOf(adjuster)+1)%adjusters.size());
        }

        double val = adjuster.update();
        PIDFCoefficients pidf = follower.getConstants().getCoefficientsTranslationalPIDF();
        telemetry.addData("PIDF", "%s = %.6f", adjuster.getName(), val);
        telemetry.addData("Use D_Pad to change parameter value.","");
        telemetry.addData("Rt Bumper for next parameter","");

    }
}