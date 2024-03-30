package org.murraybridgebunyips.bunyipslib.tasks;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.murraybridgebunyips.bunyipslib.BunyipsSubsystem;
import org.murraybridgebunyips.bunyipslib.Controls;
import org.murraybridgebunyips.bunyipslib.EmergencyStop;
import org.murraybridgebunyips.bunyipslib.pid.PIDController;
import org.murraybridgebunyips.bunyipslib.roadrunner.drive.RoadRunnerDrive;
import org.murraybridgebunyips.bunyipslib.tasks.bases.ForeverTask;
import org.murraybridgebunyips.bunyipslib.vision.data.AprilTagData;
import org.murraybridgebunyips.bunyipslib.vision.processors.AprilTag;

import java.util.List;
import java.util.Optional;

/**
 * Task to align to an AprilTag.
 *
 * @param <T> the drivetrain to use (must implement RoadRunnerDrive for X pose forward info/FCD)
 * @author Lucas Bubner, 2024
 */
@Config
public class AlignToAprilTagTask<T extends BunyipsSubsystem> extends ForeverTask {
    /**
     * PID coefficients for the alignment controller.
     */
    public static PIDCoefficients PID = new PIDCoefficients();

    private final RoadRunnerDrive drive;
    private final AprilTag at;
    private final Gamepad gamepad;
    private final PIDController controller;

    /**
     * @param gamepad    the gamepad to use for input
     * @param drive      the drivetrain to use
     * @param at         the AprilTag processor to use
     * @param controller the PID controller to use for aligning to a target
     */
    public AlignToAprilTagTask(Gamepad gamepad, T drive, AprilTag at, PIDController controller) {
        super(drive, false);
        if (!(drive instanceof RoadRunnerDrive))
            throw new EmergencyStop("AlignToAprilTagTask must be used with a drivetrain with X forward Pose/IMU info");
        this.drive = (RoadRunnerDrive) drive;
        this.at = at;
        this.gamepad = gamepad;
        this.controller = controller;
        controller.updatePID(PID);
    }

    @Override
    protected void init() {
        if (!at.isAttached())
            throw new RuntimeException("Vision processor was initialised without being attached to the vision system");
    }

    @Override
    protected void periodic() {
        // FtcDashboard live tuning
        controller.setPID(PID);

        Pose2d pose = Controls.makeRobotPose(gamepad.left_stick_x, gamepad.left_stick_y, gamepad.right_stick_x);

        List<AprilTagData> data = at.getData();

        Optional<AprilTagData> target = data.stream().filter(p -> p.getId() == 2).findFirst();
        if (!target.isPresent()) {
            drive.setWeightedDrivePower(pose);
            return;
        }
        // TODO: Optimise, may need to use a different error calculation method
        drive.setWeightedDrivePower(
                new Pose2d(
                        pose.getX(),
                        pose.getY(),
                        -controller.calculate(target.get().getYaw(), 0.0)
                )
        );
    }

    @Override
    protected void onFinish() {
//        drive.setSpeedUsingController(0, 0, 0);
    }
}
