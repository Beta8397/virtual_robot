package org.firstinspires.ftc.teamcode.pedropathing.tuning.foresight;


import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.math.Pose;
import com.pedropathing.utils.Utils;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.pedropathing.tuning.Constants;
import org.firstinspires.ftc.teamcode.pedropathing.tuning.TuningOpMode;

import java.util.ArrayList;
import java.util.List;

@Disabled
@Autonomous(group = "PedroTune")
public class HeadingTuner extends TuningOpMode {

    private static final double POWER = 0.4;
    private static final double RUNTIME = 1.2;
    private static final int SAMPLES = 15;
    public static double ALPHA = 18.25;

    private double tau;
    private double K;
    private double kV;
    private double kA;
    private double vMax = 0;
    private final List<Double> times = new ArrayList<>();
    private final List<Double> velocities = new ArrayList<>();
    private final ElapsedTime timer = new ElapsedTime();
    private boolean done = false;
    private double lastTime = 0.0;

    @Override
    public void runTuningOpMode() throws InterruptedException {

        localizer.setPose(Pose.zero());
        localizer.update();

        times.clear();
        velocities.clear();
        done = false;
        vMax = 0;
        lastTime = 0.0;

        Thread.sleep(1000);
        waitForStart();
        localizer.setPose(Pose.zero());
        localizer.update();
        timer.reset();
        lastTime = timer.seconds();
        drivetrain.drive(new DrivePowers(0.0, 0.0, POWER), false);

        while (!done && opModeIsActive()) {
            double now = timer.seconds();
            double dt = now - lastTime;
            if (dt <= 0) dt = 1e-6;
            lastTime = now;

            localizer.update();

            if (!done) {
                times.add(timer.seconds());

                double turnVel = Math.abs(localizer.velocity().omega);
                vMax = Math.max(vMax, turnVel / POWER);

                velocities.add(turnVel);

                if (timer.seconds() >= RUNTIME) {
                    done = true;
                    systemIdentification();
                    drivetrain.drive(new DrivePowers(0.0, 0.0, 0.0), false);
                } else {
                    drivetrain.drive(new DrivePowers(0.0, 0.0, POWER), false);
                }
            }
        }

        drivetrain.drive(new DrivePowers(0.0, 0.0, 0.0), true);

        double headingKP = calculatekP(ALPHA);

        Constants.tuningHeading = headingKP;

        while (opModeIsActive()){
            telemetry.addData("Headind kP", headingKP);
            telemetry.update();
        }
    }

    private double calculatekP(double alpha) {
        kV = 1 / K;
        kA = tau / K;
        return tau * alpha * alpha / K;
    }

    private void systemIdentification() {
        int N = times.size();
        if (N < 4) {
            throw new IllegalArgumentException("Failed calibration.");
        }

        int start = Math.max(0, N - SAMPLES);
        double samples = N - start;
        double sum = 0;
        for (int i = start; i < N; i++) sum += velocities.get(i);
        double A = sum / samples;
        this.K = A / POWER;

        List<Double> y = new ArrayList<>();
        List<Double> x = new ArrayList<>();
        for (int i = 0; i < N; i++) {
            double vel = velocities.get(i) / POWER;
            if (vel > 0.8 * K) continue;
            if (vel < 0.1 * K) continue;
            y.add(Math.log(K - vel));
            x.add(times.get(i));
        }
        double[] linReg = Utils.linearFit(
                x.toArray(new Double[0]),
                y.toArray(new Double[0])
        );
        if (linReg[1] == 0) throw new IllegalArgumentException("Failed calibration.");
        this.tau = -1.0/linReg[1];
    }
}
