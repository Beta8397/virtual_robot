package org.murraybridgebunyips.bunyipslib.tasks;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.Range;

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
 * Task to move to and align to a contour using the vision system.
 *
 * @author Lucas Bubner, 2024
 */
@Config
public class MoveToContourTask extends Task {
    /**
     * The PIDF coefficients for the translational controller.
     */
    public static PIDFCoefficients TRANSLATIONAL_PIDF = new PIDFCoefficients();
    /**
     * The PIDF coefficients for the rotational controller.
     */
    public static PIDFCoefficients ROTATIONAL_PIDF = new PIDFCoefficients();
    /**
     * The target position of the pixel on the camera pitch axis.
     */
    public static double PITCH_TARGET = 0.0;

    private final RoadRunnerDrive drive;
    private final Processor<ContourData> processor;
    private final PIDFController translationController;
    private final PIDFController rotationController;
    private DoubleSupplier x;
    private DoubleSupplier y;
    private DoubleSupplier r;

    /**
     * TeleOp constructor.
     *
     * @param xSupplier             x (strafe) value
     * @param ySupplier             y (forward) value
     * @param rSupplier             r (rotate) value
     * @param drive                 the drivetrain to use, must be a RoadRunnerDrive
     * @param processor             the vision processor to use
     * @param translationController the PID controller for the translational movement
     * @param rotationController    the PID controller for the rotational movement
     */
    public MoveToContourTask(DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier rSupplier, BunyipsSubsystem drive, Processor<ContourData> processor, PIDFController translationController, PIDFController rotationController) {
        super(INFINITE_TIMEOUT, drive, false);
        if (!(drive instanceof RoadRunnerDrive))
            throw new EmergencyStop("MoveToContourTask must be used with a drivetrain with X forward Pose/IMU info");
        this.drive = (RoadRunnerDrive) drive;
        this.processor = processor;
        x = xSupplier;
        y = ySupplier;
        r = rSupplier;
        this.translationController = translationController;
        this.rotationController = rotationController;
        translationController.updatePIDF(TRANSLATIONAL_PIDF);
        rotationController.updatePIDF(ROTATIONAL_PIDF);
        withName("Move to Contour");
    }

    /**
     * TeleOp constructor using a default Mecanum binding.
     *
     * @param driver                the gamepad to use for driving
     * @param drive                 the drivetrain to use, must be a RoadRunnerDrive
     * @param processor             the vision processor to use
     * @param translationController the PID controller for the translational movement
     * @param rotationController    the PID controller for the rotational movement
     */
    public MoveToContourTask(Gamepad driver, BunyipsSubsystem drive, Processor<ContourData> processor, PIDFController translationController, PIDFController rotationController) {
        this(() -> driver.left_stick_x, () -> driver.left_stick_y, () -> driver.right_stick_x, drive, processor, translationController, rotationController);
    }

    /**
     * Autonomous constructor.
     *
     * @param timeout               the maximum timeout for the task
     * @param drive                 the drivetrain to use, must be a RoadRunnerDrive
     * @param processor             the vision processor to use
     * @param translationController the PID controller for the translational movement
     * @param rotationController    the PID controller for the rotational movement
     */
    public MoveToContourTask(Measure<Time> timeout, BunyipsSubsystem drive, Processor<ContourData> processor, PIDFController translationController, PIDFController rotationController) {
        super(timeout, drive, false);
        if (!(drive instanceof RoadRunnerDrive))
            throw new EmergencyStop("MoveToContourTask must be used with a drivetrain with X forward Pose/IMU info");
        this.drive = (RoadRunnerDrive) drive;
        this.processor = processor;
        this.translationController = translationController;
        this.rotationController = rotationController;
        translationController.updatePIDF(TRANSLATIONAL_PIDF);
        rotationController.updatePIDF(ROTATIONAL_PIDF);
        withName("Move to Contour");
    }

    /**
     * Set the pitch target of where the pixel should be on the camera. 0.0 is the middle.
     *
     * @param pitchTarget the target pitch to move to
     * @return the task
     */
    public MoveToContourTask withPitchTarget(double pitchTarget) {
        PITCH_TARGET = pitchTarget;
        return this;
    }

    @Override
    protected void init() {
        if (!processor.isAttached())
            throw new RuntimeException("Vision processor was initialised without being attached to the vision system");
    }

    @Override
    protected void periodic() {
        // FtcDashboard live tuning
        translationController.setPIDF(TRANSLATIONAL_PIDF);
        rotationController.setPIDF(ROTATIONAL_PIDF);

        Pose2d pose = new Pose2d(0, 0, 0);
        if (x != null)
            pose = Controls.makeRobotPose(x.getAsDouble(), y.getAsDouble(), r.getAsDouble());

        List<ContourData> data = processor.getData();
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
        return x == null && translationController.atSetPoint() && rotationController.atSetPoint();
    }
}
