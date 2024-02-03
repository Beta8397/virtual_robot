package org.murraybridgebunyips.bunyipslib.tasks;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.murraybridgebunyips.bunyipslib.BunyipsSubsystem;
import org.murraybridgebunyips.bunyipslib.Controller;
import org.murraybridgebunyips.bunyipslib.EmergencyStop;
import org.murraybridgebunyips.bunyipslib.pid.PIDController;
import org.murraybridgebunyips.bunyipslib.roadrunner.drive.RoadRunnerDrive;
import org.murraybridgebunyips.bunyipslib.tasks.bases.RunForeverTask;
import org.murraybridgebunyips.bunyipslib.vision.Vision;
import org.murraybridgebunyips.bunyipslib.vision.processors.YCbCrColourThreshold;

/**
 * Task to align to a pixel using the vision system.
 *
 * @param <T> the drivetrain to use (must implement RoadRunnerDrive for X pose forward info/FCD)
 * @author Lucas Bubner, 2024
 */
@Config
public class AlignToPixelTask<T extends BunyipsSubsystem> extends RunForeverTask {
    public static double kP;
    public static double kI;
    public static double kD;

    private final RoadRunnerDrive drive;
    private final Vision vision;
    private final YCbCrColourThreshold processor;
    private final Gamepad gamepad;
    private final PIDController controller;

    public AlignToPixelTask(Gamepad gamepad, T drive, Vision vision, YCbCrColourThreshold processor, PIDController controller) {
        super(drive, false);
        if (!(drive instanceof RoadRunnerDrive))
            throw new EmergencyStop("AlignToPixelTask must be used with a drivetrain with X forward Pose/IMU info");
        this.drive = (RoadRunnerDrive) drive;
        this.vision = vision;
        this.processor = processor;
        this.gamepad = gamepad;
        this.controller = controller;
    }

    @Override
    public void init() {
        if (!vision.isInitialised())
            vision.init(processor);
        if (!vision.getAttachedProcessors().contains(processor))
            throw new EmergencyStop("Vision processor was initialised without being attached to the vision system");
        vision.start(processor);
    }

    @Override
    public void periodic() {
        // FtcDashboard live tuning
        controller.setPID(kP, kI, kD);

        Pose2d pose = Controller.makeRobotPose(gamepad.left_stick_x, gamepad.left_stick_y, gamepad.right_stick_x);

        if (processor.getData().size() > 0) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            pose.getX(),
                            pose.getY(),
                            -controller.calculate(processor.getData().get(0).getCenterX(), 0.5)
                    )
            );
        } else {
            // Consider using FCD once testing is done
            drive.setWeightedDrivePower(pose);
        }
    }

    @Override
    public void onFinish() {
//        drive.setSpeedUsingController(0, 0, 0);
        vision.stop(processor);
    }
}
