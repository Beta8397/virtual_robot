package org.murraybridgebunyips.bunyipslib.tasks;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.Range;

import org.murraybridgebunyips.bunyipslib.BunyipsSubsystem;
import org.murraybridgebunyips.bunyipslib.Controls;
import org.murraybridgebunyips.bunyipslib.EmergencyStop;
import org.murraybridgebunyips.bunyipslib.external.pid.PIDController;
import org.murraybridgebunyips.bunyipslib.external.units.Measure;
import org.murraybridgebunyips.bunyipslib.external.units.Time;
import org.murraybridgebunyips.bunyipslib.roadrunner.drive.RoadRunnerDrive;
import org.murraybridgebunyips.bunyipslib.tasks.bases.Task;
import org.murraybridgebunyips.bunyipslib.vision.data.ContourData;
import org.murraybridgebunyips.bunyipslib.vision.processors.MultiColourThreshold;

import java.util.List;

/**
 * Task to move to and align to a contour using the vision system.
 *
 * @param <T> the drivetrain to use (must implement RoadRunnerDrive for X pose forward info/FCD)
 * @author Lucas Bubner, 2024
 */
@Config
public class MoveToContourTask<T extends BunyipsSubsystem> extends Task {
    /**
     * The PID coefficients for the translational controller.
     */
    public static PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients();
    /**
     * The PID coefficients for the rotational controller.
     */
    public static PIDCoefficients ROTATIONAL_PID = new PIDCoefficients();
    /**
     * The target position of the pixel on the camera pitch axis.
     */
    public static double PITCH_TARGET = 0.0;

    private final RoadRunnerDrive drive;
    private final MultiColourThreshold processors;
    private final PIDController translationController;
    private final PIDController rotationController;
    private Gamepad gamepad;

    /**
     * TeleOp constructor.
     *
     * @param gamepad               the gamepad to use for manual control
     * @param drive                 the drivetrain to use
     * @param processors            the vision processors to use
     * @param translationController the PID controller for the translational movement
     * @param rotationController    the PID controller for the rotational movement
     */
    public MoveToContourTask(Gamepad gamepad, T drive, MultiColourThreshold processors, PIDController translationController, PIDController rotationController) {
        super(INFINITE_TIMEOUT, drive, false);
        if (!(drive instanceof RoadRunnerDrive))
            throw new EmergencyStop("MoveToContourTask must be used with a drivetrain with X forward Pose/IMU info");
        this.drive = (RoadRunnerDrive) drive;
        this.processors = processors;
        this.gamepad = gamepad;
        this.translationController = translationController;
        this.rotationController = rotationController;
        translationController.updatePID(TRANSLATIONAL_PID);
        rotationController.updatePID(ROTATIONAL_PID);
    }

    /**
     * Autonomous constructor.
     *
     * @param timeout               the maximum timeout for the task
     * @param drive                 the drivetrain to use
     * @param processors            the vision processors to use
     * @param translationController the PID controller for the translational movement
     * @param rotationController    the PID controller for the rotational movement
     */
    public MoveToContourTask(Measure<Time> timeout, T drive, MultiColourThreshold processors, PIDController translationController, PIDController rotationController) {
        super(timeout, drive, false);
        if (!(drive instanceof RoadRunnerDrive))
            throw new EmergencyStop("MoveToContourTask must be used with a drivetrain with X forward Pose/IMU info");
        this.drive = (RoadRunnerDrive) drive;
        this.processors = processors;
        this.translationController = translationController;
        this.rotationController = rotationController;
        translationController.updatePID(TRANSLATIONAL_PID);
        rotationController.updatePID(ROTATIONAL_PID);
    }

    /**
     * Set the pitch target of where the pixel should be on the camera. 0.0 is the middle.
     *
     * @param pitchTarget the target pitch to move to
     * @return the task
     */
    public MoveToContourTask<T> withPitchTarget(double pitchTarget) {
        PITCH_TARGET = pitchTarget;
        return this;
    }

    @Override
    protected void init() {
        if (!processors.isAttached())
            throw new RuntimeException("Vision processor was initialised without being attached to the vision system");
    }

    @Override
    protected void periodic() {
        // FtcDashboard live tuning
        translationController.setPID(TRANSLATIONAL_PID);
        rotationController.setPID(ROTATIONAL_PID);

        Pose2d pose = new Pose2d(0, 0, 0);
        if (gamepad != null)
            pose = Controls.makeRobotPose(gamepad.left_stick_x, gamepad.left_stick_y, gamepad.right_stick_x);

        List<ContourData> data = processors.getData();
        ContourData biggestContour = ContourData.getLargest(data);

        if (biggestContour != null) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -translationController.calculate(biggestContour.getPitch(), Range.clip(PITCH_TARGET, -1.0, 1.0)),
                            pose.getY(),
                            rotationController.calculate(biggestContour.getYaw(), 0.0)
                    )
            );
        } else {
            drive.setWeightedDrivePower(pose);
        }
    }

    @Override
    protected void onFinish() {
//        drive.setSpeedUsingController(0, 0, 0);
    }

    @Override
    protected boolean isTaskFinished() {
        return gamepad == null && translationController.atSetPoint() && rotationController.atSetPoint();
    }
}
