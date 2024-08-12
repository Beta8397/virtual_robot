package org.murraybridgebunyips.bunyipslib.roadrunner.drive.tuning.opmodes;

import static org.murraybridgebunyips.bunyipslib.Text.round;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.MovingStatistics;
import com.qualcomm.robotcore.util.RobotLog;

import org.murraybridgebunyips.bunyipslib.DualTelemetry;
import org.murraybridgebunyips.bunyipslib.TriConsumer;
import org.murraybridgebunyips.bunyipslib.roadrunner.drive.RoadRunnerDrive;
import org.murraybridgebunyips.bunyipslib.roadrunner.drive.localizers.ThreeWheelLocalizer;

/**
 * This routine determines the effective forward offset for the lateral tracking wheel.
 * The procedure executes a point turn at a given angle for a certain number of trials,
 * along with a specified delay in milliseconds. The purpose of this is to track the
 * change in the y position during the turn. The offset, or distance, of the lateral tracking
 * wheel from the center or rotation allows the wheel to spin during a point turn, leading
 * to an incorrect measurement for the y position. This creates an arc around around
 * the center of rotation with an arc length of change in y and a radius equal to the forward
 * offset. We can compute this offset by calculating (change in y position) / (change in heading)
 * which returns the radius if the angle (change in heading) is in radians. This is based
 * on the arc length formula of length = theta * radius.
 * <p>
 * To run this routine, simply adjust the desired angle and specify the number of trials
 * and the desired delay. Then, run the procedure. Once it finishes, it will print the
 * average of all the calculated forward offsets derived from the calculation. This calculated
 * forward offset is then added onto the current forward offset to produce an overall estimate
 * for the forward offset. You can run this procedure as many times as necessary until a
 * satisfactory result is produced.
 */
public class TrackingWheelForwardOffsetTuner implements TriConsumer<LinearOpMode, DualTelemetry, RoadRunnerDrive> {
    /**
     * The angle to turn for the forward offset tuning procedure.
     */
    public double ANGLE_DEGREES = 180;
    /**
     * The number of trials to run for the forward offset tuning procedure.
     */
    public int NUM_TRIALS = 5;
    /**
     * The delay in milliseconds between each trial.
     */
    public int DELAY_MILLIS = 1000;

    @Override
    public void accept(LinearOpMode opMode, DualTelemetry telemetry, RoadRunnerDrive drive) {
        // Must set localizer to a StandardTrackingWheelLocalizer, at the moment this will not run
        Localizer localizer = drive.getLocalizer();

        if (!(localizer instanceof ThreeWheelLocalizer)) {
            RobotLog.setGlobalErrorMsg("StandardTrackingWheelLocalizer is not being set in the "
                    + "drive class. Ensure that \"setLocalizer(new StandardTrackingWheelLocalizer"
                    + "(...));\" is called somewhere else.");
        }
        assert localizer instanceof ThreeWheelLocalizer;

        telemetry.add("Press play to begin the forward offset tuner");
        telemetry.add("Make sure your robot has enough clearance to turn smoothly");
        telemetry.update();

        opMode.waitForStart();

        if (opMode.isStopRequested()) return;

        telemetry.clear();
        telemetry.add("Running...");
        telemetry.update();

        MovingStatistics forwardOffsetStats = new MovingStatistics(NUM_TRIALS);
        for (int i = 0; i < NUM_TRIALS; i++) {
            drive.setPoseEstimate(new Pose2d());

            // it is important to handle heading wraparounds
            double headingAccumulator = 0;
            double lastHeading = 0;

            drive.turnAsync(Math.toRadians(ANGLE_DEGREES));

            while (!opMode.isStopRequested() && drive.isBusy()) {
                double heading = drive.getPoseEstimate().getHeading();
                headingAccumulator += Angle.norm(heading - lastHeading);
                lastHeading = heading;

                drive.update();
            }

            double forwardOffset = ((ThreeWheelLocalizer) localizer).getCoefficients().FORWARD_OFFSET +
                    drive.getPoseEstimate().getY() / headingAccumulator;
            forwardOffsetStats.add(forwardOffset);

            opMode.sleep(DELAY_MILLIS);
        }

        telemetry.clear();
        telemetry.add("Tuning complete");
        telemetry.add("Effective forward offset = % (SE = %)",
                round(forwardOffsetStats.getMean(), 2),
                round(forwardOffsetStats.getStandardDeviation() / Math.sqrt(NUM_TRIALS), 3));
        telemetry.update();

        while (!opMode.isStopRequested()) opMode.idle();
    }
}