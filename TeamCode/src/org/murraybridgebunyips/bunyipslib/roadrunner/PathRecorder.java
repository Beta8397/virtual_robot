package org.murraybridgebunyips.bunyipslib.roadrunner;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.murraybridgebunyips.bunyipslib.BunyipsOpMode;
import org.murraybridgebunyips.bunyipslib.Controls;
import org.murraybridgebunyips.bunyipslib.Dbg;
import org.murraybridgebunyips.bunyipslib.EmergencyStop;
import org.murraybridgebunyips.bunyipslib.roadrunner.drive.RoadRunnerDrive;

import java.util.ArrayList;

/**
 * PathRecorder - a macro-like RoadRunner trajectory generator from user input
 * <p>
 * To use this class, you must be using a RoadRunner drive that can provide pose estimates,
 * and proper starting pose data. This class will record user input and generate a trajectory
 * from that input. This trajectory can then be used to generate a RoadRunner trajectory.
 * <p>
 * The paths generated from this class will be of a continuous spline type, and therefore will not take
 * advantage of strafing or turning optimizations, and can be very inaccurate. This class is intended
 * for use in simple differential-like paths that do not require complex motion.
 * Ensure to verify paths with a visualiser such as BunyipsLib_RRPathVisualiser.
 * <p>
 * You will need to make a new OpMode that extends this class.
 *
 * @author Lucas Bubner, 2024
 */
public abstract class PathRecorder extends BunyipsOpMode {
    private final ArrayList<Pose2d> path = new ArrayList<>();
    private final ElapsedTime timer = new ElapsedTime();
    private RoadRunnerDrive drive;
    private int snapshotDuration;
    private Vector2d deltaThreshold;
    private Pose2d startPose;
    private Pose2d currentPose;

    /**
     * Initialise your config here.
     */
    protected abstract void configureRobot();

    /**
     * Set/configure the drive that will be used to record the path.
     *
     * @return the drive that will be used to record the path, will be called after configureRobot()
     */
    protected abstract RoadRunnerDrive setDrive();

    /**
     * Set the starting pose of the robot.
     *
     * @return the starting pose of the robot, this should be in the inches coordinate system used by RoadRunner/FtcDashboard/RRPathGen
     */
    protected abstract Pose2d setStartPose();

    /**
     * Set the delta threshold for the robot to move before a new snapshot is taken.
     *
     * @return a pose representing the delta threshold, in inches
     */
    protected abstract Vector2d setDeltaThreshold();

    /**
     * Set the duration at which Pose snapshots will be taken (a path will be generated from these snapshots).
     *
     * @return the duration at which Pose snapshots will be taken, in milliseconds
     */
    protected abstract int setSnapshotDurationMs();

    @Override
    protected final void onInit() {
        configureRobot();
        // Double check as configureRobot() may have already configured these things for us
        if (drive == null)
            drive = setDrive();
        if (drive == null)
            throw new EmergencyStop("Drive not set in PathRecorder");

        if (startPose == null)
            startPose = setStartPose();
        if (startPose == null)
            throw new EmergencyStop("Start pose not set in PathRecorder");

        drive.setPoseEstimate(startPose);
        currentPose = startPose;

        if (snapshotDuration == 0)
            snapshotDuration = Math.abs(setSnapshotDurationMs());
        if (snapshotDuration == 0)
            throw new EmergencyStop("Snapshot duration must be not zero");

        if (deltaThreshold == null)
            deltaThreshold = setDeltaThreshold();
        if (deltaThreshold == null)
            throw new EmergencyStop("Delta threshold not set in PathRecorder");
    }

    @Override
    protected final boolean onInitLoop() {
        // no-op
        return true;
    }

    @Override
    protected final void onInitDone() {
        // no-op
    }

    @Override
    protected final void onStart() {
        timer.reset();
    }

    @Override
    protected final void activeLoop() {
        telemetry.add("PoseRecorder is recording at %ms intervals...", snapshotDuration);
        telemetry.add("Current pose: %", currentPose.toString());
        telemetry.add("Snapshot count: %", path.size());
        telemetry.add("Press B to stop recording.\n");
        if (gamepad1.b) {
            drive.stop();
            captureSnapshot();
            processData();
            finish();
        }
        drive.setWeightedDrivePower(
                Controls.makeRobotPose(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x)
        );
        currentPose = drive.getPoseEstimate();
        if (timer.milliseconds() >= snapshotDuration) {
            captureSnapshot();
        }
        drive.update();
    }

    private void captureSnapshot() {
        if ((!path.isEmpty() && currentPose.equals(path.get(path.size() - 1))) || (path.isEmpty() && currentPose.equals(startPose))) {
            Dbg.log(getClass(), "Skipping duplicate pose %", currentPose.toString());
            timer.reset();
            return;
        }
        if (!path.isEmpty() && currentPose.minus(path.get(path.size() - 1)).vec().norm() < deltaThreshold.norm()) {
            Dbg.log(getClass(), "Skipping pose %, delta % < %", currentPose.toString(), currentPose.minus(path.get(path.size() - 1)).vec().norm(), deltaThreshold.norm());
            timer.reset();
            return;
        }
        path.add(new Pose2d(currentPose.getX(), currentPose.getY(), getDeltaHeading()));
        Dbg.log(getClass(), "Snapshotting pose % -> %", path.size(), currentPose.toString());
        timer.reset();
    }

    private double getDeltaHeading() {
        double deltaHeading = currentPose.getHeading();
        if (!path.isEmpty()) {
            // We cannot rely on the robot heading otherwise the path when strafing will
            // rely on the robot's heading, which will move the robot over by dy, rotate it, and then move it back
            // to the original position. This is not what we want. We will use some trigonometry to calculate the
            // heading of the path relative to the last point in the path.
            // d_heading = tan^-1(dy/dx)
            deltaHeading = Math.atan2(
                    currentPose.getY() - path.get(path.size() - 1).getY(),
                    currentPose.getX() - path.get(path.size() - 1).getX()
            );
        }
        return deltaHeading;
    }

    private void processData() {
        Dbg.log("Processing data of % poses...", path.size());
        StringBuilder sb = new StringBuilder();
        sb.append("makeTrajectory(new Pose2d(")
                .append(startPose.getX())
                .append(", ")
                .append(startPose.getY())
                .append(", ")
                .append(startPose.getHeading())
                .append("))")
                .append("\n");
        for (Pose2d pose : path) {
            sb.append("    .splineTo(new Vector2d(")
                    .append(pose.getX())
                    .append(", ")
                    .append(pose.getY())
                    .append("), ")
                    .append(pose.getHeading())
                    .append(")\n");
        }
        sb.append("    .build();");

        // DS + FtcDashboard
        telemetry.clearAll();
        telemetry.addRetained("Generated path:\n" + sb);
        telemetry.update();

        // Logcat
        Dbg.log("Generated path:\n" + sb);
    }

    @Override
    protected final void onStop() {
        // no-op
    }
}
