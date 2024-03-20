package org.murraybridgebunyips.bunyipslib.tasks;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.murraybridgebunyips.bunyipslib.BunyipsSubsystem;
import org.murraybridgebunyips.bunyipslib.Controller;
import org.murraybridgebunyips.bunyipslib.EmergencyStop;
import org.murraybridgebunyips.bunyipslib.pid.PIDController;
import org.murraybridgebunyips.bunyipslib.roadrunner.drive.RoadRunnerDrive;
import org.murraybridgebunyips.bunyipslib.tasks.bases.Task;
import org.murraybridgebunyips.bunyipslib.vision.data.ContourData;
import org.murraybridgebunyips.bunyipslib.vision.processors.MultiColourThreshold;

import java.util.List;

/**
 * Task to align to a contour using the vision system.
 *
 * @param <T> the drivetrain to use (must implement RoadRunnerDrive for X pose forward info/FCD)
 * @author Lucas Bubner, 2024
 */
@Config
public class AlignToContourTask<T extends BunyipsSubsystem> extends Task {
    /**
     * PID coefficients for the alignment controller.
     */
    public static PIDCoefficients PID = new PIDCoefficients();

    private final RoadRunnerDrive drive;
    private final MultiColourThreshold processors;
    private final PIDController controller;
    private Gamepad gamepad;

    /**
     * TeleOp constructor
     *
     * @param gamepad    the gamepad to use for input
     * @param drive      the drivetrain to use
     * @param processors the vision processor to use
     * @param controller the PID controller to use for aligning to a target
     */
    public AlignToContourTask(Gamepad gamepad, T drive, MultiColourThreshold processors, PIDController controller) {
        super(0, drive, false);
        if (!(drive instanceof RoadRunnerDrive))
            throw new EmergencyStop("AlignToContourTask must be used with a drivetrain with X forward Pose/IMU info");
        this.drive = (RoadRunnerDrive) drive;
        this.processors = processors;
        this.gamepad = gamepad;
        this.controller = controller;
        controller.updatePID(PID);
    }

    /**
     * Autonomous constructor
     *
     * @param timeout    the maximum time in seconds to run the task for
     * @param drive      the drivetrain to use
     * @param processors the vision processor to use
     * @param controller the PID controller to use for aligning to a target
     */
    public AlignToContourTask(double timeout, T drive, MultiColourThreshold processors, PIDController controller) {
        super(timeout, drive, false);
        if (!(drive instanceof RoadRunnerDrive))
            throw new EmergencyStop("AlignToContourTask must be used with a drivetrain with X forward Pose/IMU info");
        this.drive = (RoadRunnerDrive) drive;
        this.processors = processors;
        this.controller = controller;
        controller.updatePID(PID);
    }

    @Override
    protected void init() {
        if (!processors.isAttached())
            throw new RuntimeException("Vision processor was initialised without being attached to the vision system");
    }

    @Override
    protected void periodic() {
        // FtcDashboard live tuning
        controller.setPID(PID);

        Pose2d pose = new Pose2d(0, 0, 0);
        if (gamepad != null)
            pose = Controller.makeRobotPose(gamepad.left_stick_x, gamepad.left_stick_y, gamepad.right_stick_x);

        List<ContourData> data = processors.getData();
        ContourData biggestContour = ContourData.getLargest(data);

        if (biggestContour != null) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            pose.getX(),
                            pose.getY(),
                            controller.calculate(biggestContour.getYaw(), 0.0)
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
        return gamepad == null && controller.atSetPoint();
    }
}
