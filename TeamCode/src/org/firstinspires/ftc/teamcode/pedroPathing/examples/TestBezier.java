package org.firstinspires.ftc.teamcode.pedroPathing.examples;


import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.tuning.tuning_util.Adjuster;

import java.util.ArrayList;
import java.util.List;

@Autonomous(group="Pedro")
public class TestBezier extends OpMode {

    private Follower follower;

    private PathChain forwards;

    private PathChain backwards;

    boolean forward = true;

    boolean turning = false;

    List<Adjuster> adjusters;
    Adjuster adjuster;

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
//                .setReversed()
                .build();

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
            if (turning) {
                turning = false;
                if (forward) {
                    forward = false;
                    follower.followPath(backwards);
                } else {
                    forward = true;
                    follower.followPath(forwards);
                }
            } else {
                turning = true;
                double targetAngle = Math.toRadians(forward? 270 : 90);
                follower.turnTo(targetAngle);
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