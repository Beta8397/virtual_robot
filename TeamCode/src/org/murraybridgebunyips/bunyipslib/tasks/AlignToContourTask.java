package org.murraybridgebunyips.bunyipslib.tasks;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.murraybridgebunyips.bunyipslib.BunyipsSubsystem;
import org.murraybridgebunyips.bunyipslib.Controls;
import org.murraybridgebunyips.bunyipslib.EmergencyStop;
import org.murraybridgebunyips.bunyipslib.external.pid.PIDFController;
import org.murraybridgebunyips.bunyipslib.external.units.Measure;
import org.murraybridgebunyips.bunyipslib.external.units.Time;
import org.murraybridgebunyips.bunyipslib.roadrunner.drive.RoadRunnerDrive;
import org.murraybridgebunyips.bunyipslib.tasks.bases.Task;
import org.murraybridgebunyips.bunyipslib.vision.Processor;
import org.murraybridgebunyips.bunyipslib.vision.data.ContourData;

import java.util.List;
import java.util.function.DoubleSupplier;

/**
 * Task to align to a contour using the vision system.
 *
 * @author Lucas Bubner, 2024
 */
@Config
public class AlignToContourTask extends Task {
    /**
     * PIDF coefficients for the alignment controller.
     */
    public static PIDFCoefficients PIDF = new PIDFCoefficients();

    private final RoadRunnerDrive drive;
    private final Processor<ContourData> processor;
    private final PIDFController controller;
    private DoubleSupplier x;
    private DoubleSupplier y;
    private DoubleSupplier r;

    /**
     * TeleOp constructor
     *
     * @param xSupplier  x (strafe) value
     * @param ySupplier  y (forward) value
     * @param rSupplier  r (rotate) value
     * @param drive      the drivetrain to use, must be a RoadRunnerDrive
     * @param processor  the vision processor to use
     * @param controller the PID controller to use for aligning to a target
     */
    public AlignToContourTask(DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier rSupplier, BunyipsSubsystem drive, Processor<ContourData> processor, PIDFController controller) {
        super(INFINITE_TIMEOUT, drive, false);
        if (!(drive instanceof RoadRunnerDrive))
            throw new EmergencyStop("AlignToContourTask must be used with a drivetrain with X forward Pose/IMU info");
        this.drive = (RoadRunnerDrive) drive;
        this.processor = processor;
        x = xSupplier;
        y = ySupplier;
        r = rSupplier;
        this.controller = controller;
        controller.updatePIDF(PIDF);
        withName("Align To Contour");
    }

    /**
     * TeleOp constructor using a default Mecanum binding.
     *
     * @param driver     the gamepad to use for driving
     * @param drive      the drivetrain to use, must be a RoadRunnerDrive
     * @param processors the vision processor to use
     * @param controller the PID controller to use for aligning to a target
     */
    public AlignToContourTask(Gamepad driver, BunyipsSubsystem drive, Processor<ContourData> processors, PIDFController controller) {
        this(() -> driver.left_stick_x, () -> driver.left_stick_y, () -> driver.right_stick_x, drive, processors, controller);
    }

    /**
     * Autonomous constructor
     *
     * @param timeout    the maximum time in seconds to run the task for
     * @param drive      the drivetrain to use, must be a RoadRunnerDrive
     * @param processor  the vision processor to use
     * @param controller the PID controller to use for aligning to a target
     */
    public AlignToContourTask(Measure<Time> timeout, BunyipsSubsystem drive, Processor<ContourData> processor, PIDFController controller) {
        super(timeout, drive, false);
        if (!(drive instanceof RoadRunnerDrive))
            throw new EmergencyStop("AlignToContourTask must be used with a drivetrain with X forward Pose/IMU info");
        this.drive = (RoadRunnerDrive) drive;
        this.processor = processor;
        this.controller = controller;
        controller.updatePIDF(PIDF);
        withName("Align To Contour");
    }

    @Override
    protected void init() {
        if (!processor.isAttached())
            throw new RuntimeException("Vision processor was initialised without being attached to the vision system");
    }

    @Override
    protected void periodic() {
        // FtcDashboard live tuning
        controller.setPIDF(PIDF);

        Pose2d pose = new Pose2d(0, 0, 0);
        if (x != null)
            pose = Controls.makeRobotPose(x.getAsDouble(), y.getAsDouble(), r.getAsDouble());

        List<ContourData> data = processor.getData();
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
        return x == null && controller.atSetPoint();
    }
}
