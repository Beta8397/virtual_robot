package org.firstinspires.ftc.teamcode.pedropathing.tuning.foresight;


import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.math.Pose;
import com.pedropathing.utils.Utils;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.pedropathing.tuning.TuningOpMode;

import java.util.ArrayList;
import java.util.List;

@Disabled
@Autonomous(group = "PedroTune")
public class StrafeTranslational extends TuningOpMode {
    public static double ALPHA_LARGE = 10.2;
    public static double ALPHA_SMALL = 6.2;
    private final double POWER = 0.4;
    private final double RUNTIME = 1.2;
    private final int SAMPLES = 15;

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
        drivetrain.drive(new DrivePowers(0.0, POWER, 0.0), false);

        while (!done && !isStopRequested()) {
            double now = timer.seconds();
            double dt = now - lastTime;
            if (dt <= 0) dt = 1e-6;
            lastTime = now;

            localizer.update();

            if (!done) {
                times.add(timer.seconds());

                double lateralVelocity = Math.abs(localizer.twist().toVector2D().y());
                vMax = Math.max(vMax, lateralVelocity / POWER);

                velocities.add(lateralVelocity);

                if (timer.seconds() >= RUNTIME) {
                    done = true;
                    systemIdentification();
                    drivetrain.drive(new DrivePowers(0.0, 0.0, 0.0), false);
                } else {
                    drivetrain.drive(new DrivePowers(0.0, POWER, 0.0), false);
                }
            }
        }

        drivetrain.drive(new DrivePowers(0.0, 0.0, 0.0), true);

        double kP_large = calculatekP(ALPHA_LARGE);
        double kP_small = calculatekP(ALPHA_SMALL);

        List<Double> result = List.of(kP_large, kP_small);

        while (opModeIsActive()){
            telemetry.addData("kP_large", result.get(0));
            telemetry.addData("kP_small", result.get(1));
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
