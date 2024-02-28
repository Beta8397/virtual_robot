package org.murraybridgebunyips.bunyipslib.tasks;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.murraybridgebunyips.bunyipslib.BunyipsSubsystem;
import org.murraybridgebunyips.bunyipslib.Controller;
import org.murraybridgebunyips.bunyipslib.EmergencyStop;
import org.murraybridgebunyips.bunyipslib.pid.PIDController;
import org.murraybridgebunyips.bunyipslib.roadrunner.drive.RoadRunnerDrive;
import org.murraybridgebunyips.bunyipslib.tasks.bases.ForeverTask;
import org.murraybridgebunyips.bunyipslib.vision.data.ContourData;
import org.murraybridgebunyips.bunyipslib.vision.processors.MultiColourThreshold;

import java.util.List;

/**
 * Task to move to and align to a pixel using the vision system.
 *
 * @param <T> the drivetrain to use (must implement RoadRunnerDrive for X pose forward info/FCD)
 * @author Lucas Bubner, 2024
 */
@Config
public class MoveToPixelTask<T extends BunyipsSubsystem> extends ForeverTask {
    public static double TRANSLATIONAL_kP;
    public static double TRANSLATIONAL_kI;
    public static double TRANSLATIONAL_kD;
    public static double ROTATIONAL_kP;
    public static double ROTATIONAL_kI;
    public static double ROTATIONAL_kD;

    private final RoadRunnerDrive drive;
    private final MultiColourThreshold processors;
    private final Gamepad gamepad;
    private final PIDController translationController;
    private final PIDController rotationController;

    public MoveToPixelTask(Gamepad gamepad, T drive, MultiColourThreshold processors, PIDController translationController, PIDController rotationController) {
        super(drive, false);
        if (!(drive instanceof RoadRunnerDrive))
            throw new EmergencyStop("MoveToPixelTask must be used with a drivetrain with X forward Pose/IMU info");
        this.drive = (RoadRunnerDrive) drive;
        this.processors = processors;
        this.gamepad = gamepad;
        this.translationController = translationController;
        this.rotationController = rotationController;
        TRANSLATIONAL_kP = translationController.getP();
        TRANSLATIONAL_kI = translationController.getI();
        TRANSLATIONAL_kD = translationController.getD();
        ROTATIONAL_kP = rotationController.getP();
        ROTATIONAL_kI = rotationController.getI();
        ROTATIONAL_kD = rotationController.getD();
    }

    @Override
    public void init() {
        if (!processors.isAttached())
            throw new RuntimeException("Vision processor was initialised without being attached to the vision system");
    }

    @Override
    public void periodic() {
        // FtcDashboard live tuning
        translationController.setPID(TRANSLATIONAL_kP, TRANSLATIONAL_kI, TRANSLATIONAL_kD);
        rotationController.setPID(ROTATIONAL_kP, ROTATIONAL_kI, ROTATIONAL_kD);

        Pose2d pose = Controller.makeRobotPose(gamepad.left_stick_x, gamepad.left_stick_y, gamepad.right_stick_x);

        List<ContourData> data = processors.getData();
        ContourData biggestContour = ContourData.getLargest(data);

        if (biggestContour != null) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -translationController.calculate(biggestContour.getPitch(), 0.0),
                            pose.getY(),
                            rotationController.calculate(biggestContour.getYaw(), 0.0)
                    )
            );
        } else {
            drive.setWeightedDrivePower(pose);
        }
    }

    @Override
    public void onFinish() {
//        drive.setSpeedUsingController(0, 0, 0);
    }
}
