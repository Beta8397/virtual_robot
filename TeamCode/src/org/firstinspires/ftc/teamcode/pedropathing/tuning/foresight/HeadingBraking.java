package org.firstinspires.ftc.teamcode.pedropathing.tuning.foresight;


import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.math.Pose;
import com.pedropathing.utils.Angle;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.pedropathing.tuning.Constants;
import org.firstinspires.ftc.teamcode.pedropathing.tuning.TuningOpMode;
import com.pedropathing.utils.Utils;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

@Disabled
@Autonomous(group = "PedroTune")
public class HeadingBraking extends TuningOpMode {
    private static double[] POWERS;
    public static double MAX_BRAKE_TIME = 3; //seconds, the robot shouldn't take longer than this to brake

    public static int trials = 12;
    public static double maxPower = 1;
    public static double minPower = 0.2;
    public static double bias = 1.5; // how much it favors doing trials with higher powers
    public static double brakingPower = 0.001;

    private final ElapsedTime timer = new ElapsedTime();

    private final List<double[]> velocityToBrakingDistance = new ArrayList<>();
    private State state = State.DRIVE;
    private int iteration = 0;
    private int direction;
    private double power;

    private double startHeading;
    private double measuredVelocity;
    private double totalHeading;
    private double previousHeading;

    @Override
    public void runTuningOpMode() throws InterruptedException {

        localizer.setPose(Pose.zero());
        localizer.update();

        List<Double> coefficients = Collections.emptyList();

        POWERS = biasedGradient(trials, maxPower, minPower, bias);

        Thread.sleep(1000);
        waitForStart();
        localizer.setPose(Pose.zero());
        localizer.update();
        timer.reset();

        while (state != State.DONE && opModeIsActive()) {
            localizer.update();
            double currentHeading = localizer.pose().heading();
            totalHeading += Angle.normalizeSigned(currentHeading - previousHeading);
            previousHeading = currentHeading;

            direction = (iteration % 2 == 0) ? 1 : -1;
            if (iteration < POWERS.length) {
                power = POWERS[iteration];
            }

            switch (state) {
                case DRIVE: {
                    if (timer.seconds() > 2) {
                        startHeading = totalHeading;
                        measuredVelocity = Math.abs(localizer.velocity().omega);

                        drivetrain.drive(new DrivePowers(0.0, 0.0, -brakingPower * direction), false);
                        state = State.BRAKE;
                        timer.reset();
                        break;
                    }
                    drivetrain.drive(new DrivePowers(0.0, 0.0, power * direction), false);
                    break;
                }
                case BRAKE: {
                    if (Math.abs(localizer.velocity().omega) > 0.001 && timer.seconds() < MAX_BRAKE_TIME) {
                        drivetrain.drive(new DrivePowers(0.0, 0.0, -brakingPower * direction), false);
                        break;
                    }

                    double endHeading = totalHeading;
                    double brakingDistance = Math.abs(endHeading - startHeading);

                    velocityToBrakingDistance.add(new double[]{measuredVelocity, brakingDistance});

                    iteration++;

                    if (iteration >= POWERS.length) {
                        drivetrain.stop();

                        double[] c = Utils.quadraticFit(velocityToBrakingDistance);
                        coefficients = List.of(c[0], c[1]);

                        state = State.DONE;
                    } else {
                        timer.reset();
                        state = State.DRIVE;
                    }
                    break;
                }
                case DONE: {}
            }
        }

        Constants.tuningHeadingLinear = coefficients.get(0);
        Constants.tuningHeadingQuadratic = coefficients.get(1);

        while (opModeIsActive()){
            telemetry.addData("Linear", coefficients.get(0));
            telemetry.addData("Quadratic", coefficients.get(1));
            telemetry.update();
        }
    }

    private enum State {
        DRIVE,
        BRAKE,
        DONE
    }

    private static double[] biasedGradient(
            int count,
            double max,
            double min,
            double bias
    ) {
        if (count < 2) return new double[]{  max};

        double[] values = new double[count];

        for (int i = 0; i < count; i++) {
            double t = (double) i / (count - 1);

            double curved = 1 - Math.pow(t, bias);

            values[i] = min + curved * (max - min);
        }

        return values;
    }
}
