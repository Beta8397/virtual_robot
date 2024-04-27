package org.murraybridgebunyips.bunyipslib.tasks;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.murraybridgebunyips.bunyipslib.BunyipsSubsystem;
import org.murraybridgebunyips.bunyipslib.Controls;
import org.murraybridgebunyips.bunyipslib.EmergencyStop;
import org.murraybridgebunyips.bunyipslib.external.pid.PIDController;
import org.murraybridgebunyips.bunyipslib.external.units.Measure;
import org.murraybridgebunyips.bunyipslib.external.units.Time;
import org.murraybridgebunyips.bunyipslib.roadrunner.drive.RoadRunnerDrive;
import org.murraybridgebunyips.bunyipslib.tasks.bases.Task;
import org.murraybridgebunyips.bunyipslib.vision.data.AprilTagData;
import org.murraybridgebunyips.bunyipslib.vision.processors.AprilTag;

import java.util.List;
import java.util.Optional;
import java.util.function.DoubleSupplier;

/**
 * Task to align to an AprilTag.
 *
 * @param <T> the drivetrain to use (must implement RoadRunnerDrive for X pose forward info/FCD)
 * @author Lucas Bubner, 2024
 */
@Config
public class AlignToAprilTagTask<T extends BunyipsSubsystem> extends Task {
    /**
     * PID coefficients for the alignment controller.
     */
    public static PIDCoefficients PID = new PIDCoefficients();

    private final RoadRunnerDrive drive;
    private final AprilTag at;
    private final PIDController controller;
    private DoubleSupplier x;
    private DoubleSupplier y;
    private DoubleSupplier r;

    /**
     * Autonomous constructor.
     *
     * @param timeout    the timeout for the task
     * @param drive      the drivetrain to use
     * @param at         the AprilTag processor to use
     * @param controller the PID controller to use for aligning to a target
     */
    public AlignToAprilTagTask(Measure<Time> timeout, T drive, AprilTag at, PIDController controller) {
        super(timeout, drive, false);
        if (!(drive instanceof RoadRunnerDrive))
            throw new EmergencyStop("AlignToAprilTagTask must be used with a drivetrain with X forward Pose/IMU info");
        this.drive = (RoadRunnerDrive) drive;
        this.at = at;
        this.controller = controller;
        controller.updatePID(PID);
    }

    /**
     * TeleOp constructor.
     *
     * @param xSupplier  x (strafe) value
     * @param ySupplier  y (forward) value
     * @param rSupplier  r (rotate) value
     * @param drive      the drivetrain to use
     * @param at         the AprilTag processor to use
     * @param controller the PID controller to use for aligning to a target
     */
    public AlignToAprilTagTask(DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier rSupplier, T drive, AprilTag at, PIDController controller) {
        super(INFINITE_TIMEOUT, drive, false);
        if (!(drive instanceof RoadRunnerDrive))
            throw new EmergencyStop("AlignToAprilTagTask must be used with a drivetrain with X forward Pose/IMU info");
        this.drive = (RoadRunnerDrive) drive;
        this.at = at;
        x = xSupplier;
        y = ySupplier;
        r = rSupplier;
        this.controller = controller;
        controller.updatePID(PID);
    }

    /**
     * Constructor for AlignToAprilTagTask using a default Mecanum binding.
     *
     * @param driver     The gamepad to use for driving
     * @param drive      The MecanumDrive to use for driving
     * @param at         The AprilTag processor to use
     * @param controller The PID controller to use for aligning to a target
     */
    public AlignToAprilTagTask(Gamepad driver, T drive, AprilTag at, PIDController controller) {
        this(() -> driver.left_stick_x, () -> driver.left_stick_y, () -> driver.right_stick_x, drive, at, controller);
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

        Pose2d pose = new Pose2d();
        if (x != null)
            pose = Controls.makeRobotPose(x.getAsDouble(), y.getAsDouble(), r.getAsDouble());

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

    @Override
    protected boolean isTaskFinished() {
        return x == null && controller.atSetPoint();
    }
}
