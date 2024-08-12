package org.murraybridgebunyips.bunyipslib.roadrunner.drive.tuning.opmodes;

import static org.murraybridgebunyips.bunyipslib.Text.round;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.MovingStatistics;

import org.murraybridgebunyips.bunyipslib.DualTelemetry;
import org.murraybridgebunyips.bunyipslib.TriConsumer;
import org.murraybridgebunyips.bunyipslib.roadrunner.drive.RoadRunnerDrive;

/**
 * This routine determines the effective track width. The procedure works by executing a point turn
 * with a given angle and measuring the difference between that angle and the actual angle (as
 * indicated by an external IMU/gyro, track wheels, or some other localizer).
 * <p>
 * The quotient given angle / actual angle gives a multiplicative adjustment to the estimated track width
 * (effective track width = estimated track width * given angle / actual angle). The routine repeats
 * this procedure a few times and averages the values for additional accuracy.
 * <p>
 * Note: a relatively accurate track width estimate is important or else the angular constraints will be thrown off.
 */
public class TrackWidthTuner implements TriConsumer<LinearOpMode, DualTelemetry, RoadRunnerDrive> {
    /**
     * The angle to turn for the track width tuning procedure.
     */
    public double ANGLE_DEGREES = 180;
    /**
     * The number of trials to run for the track width tuning procedure.
     */
    public int NUM_TRIALS = 5;
    /**
     * The delay in milliseconds between each trial.
     */
    public int DELAY_MILLIS = 1000;

    @Override
    public void accept(LinearOpMode opMode, DualTelemetry telemetry, RoadRunnerDrive drive) {
        // Ensure if you haven't already, set the localizer to something that doesn't depend on
        // drive encoders for computing the heading

        telemetry.add("Press play to begin the track width tuner routine");
        telemetry.add("Make sure your robot has enough clearance to turn smoothly");
        telemetry.update();

        opMode.waitForStart();

        if (opMode.isStopRequested()) return;

        telemetry.clear();
        telemetry.add("Running...");
        telemetry.update();

        MovingStatistics trackWidthStats = new MovingStatistics(NUM_TRIALS);
        for (int i = 0; i < NUM_TRIALS; i++) {
            drive.setPoseEstimate(new Pose2d());

            // it is important to handle heading wraparounds
            double headingAccumulator = 0;
            double lastHeading = 0;

            drive.turnAsync(Math.toRadians(ANGLE_DEGREES));

            while (!opMode.isStopRequested() && drive.isBusy()) {
                double heading = drive.getPoseEstimate().getHeading();
                headingAccumulator += Angle.normDelta(heading - lastHeading);
                lastHeading = heading;

                drive.update();
            }

            double trackWidth = drive.getConstants().TRACK_WIDTH * Math.toRadians(ANGLE_DEGREES) / headingAccumulator;
            trackWidthStats.add(trackWidth);

            opMode.sleep(DELAY_MILLIS);
        }

        telemetry.clear();
        telemetry.add("Tuning complete");
        telemetry.add("Effective track width = % (SE = %)",
                round(trackWidthStats.getMean(), 2),
                round(trackWidthStats.getStandardDeviation() / Math.sqrt(NUM_TRIALS), 3));
        telemetry.update();

        while (!opMode.isStopRequested()) opMode.idle();
    }
}