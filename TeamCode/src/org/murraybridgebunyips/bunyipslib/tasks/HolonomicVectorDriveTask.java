package org.murraybridgebunyips.bunyipslib.tasks;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.jetbrains.annotations.NotNull;
import org.murraybridgebunyips.bunyipslib.drive.MecanumDrive;
import org.murraybridgebunyips.bunyipslib.external.Mathf;
import org.murraybridgebunyips.bunyipslib.external.pid.PIDController;
import org.murraybridgebunyips.bunyipslib.tasks.bases.ForeverTask;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

/**
 * Gamepad drive for all holonomic drivetrains which will use a vector-based approach to drive.
 * This task is designed to be used as a default task, other tasks will override it.
 * <p>
 * Compared to {@link HolonomicDriveTask}, this task will generate a vector for the robot to follow, rather than
 * setting the powers directly from the gamepad inputs. This allows for more predictable and consistent driving,
 * as the robot will always follow the same vector for a given input (self-correcting TeleOp).
 * <p>
 * A RoadRunner drive is required for this class, as it will require the use of the pose estimate system and other
 * coefficients such as your PID. Therefore, the only supported class this task will work for is {@link MecanumDrive}.
 *
 * @author Lucas Bubner, 2024
 */
public class HolonomicVectorDriveTask extends ForeverTask {
    private final MecanumDrive drive;
    private final Supplier<Float> x;
    private final Supplier<Float> y;
    private final Supplier<Float> r;
    private final BooleanSupplier fieldCentricEnabled;

    private final PIDController xController;
    private final PIDController yController;
    private final PIDController rController;

    private double xLock;
    private double yLock;
    private double rLock;

    /**
     * Constructor for HolonomicVectorDriveTask.
     *
     * @param xSupplier           The supplier for the x-axis input
     * @param ySupplier           The supplier for the y-axis input, <i>note that this will be inverted</i>
     * @param rSupplier           The supplier for the rotation input
     * @param mecanumDrive        The MecanumDrive to use
     * @param fieldCentricEnabled A BooleanSupplier that returns whether field centric drive is enabled
     */
    public HolonomicVectorDriveTask(Supplier<Float> xSupplier, Supplier<Float> ySupplier, Supplier<Float> rSupplier, @NotNull MecanumDrive mecanumDrive, BooleanSupplier fieldCentricEnabled) {
        onSubsystem(mecanumDrive, false);
        drive = mecanumDrive;
        x = xSupplier;
        y = ySupplier;
        r = rSupplier;
        this.fieldCentricEnabled = fieldCentricEnabled;

        PIDCoefficients translationCoeffs = drive.getCoefficients().TRANSLATIONAL_PID;
        PIDCoefficients rotationCoeffs = drive.getCoefficients().HEADING_PID;
        xController = new PIDController(translationCoeffs.kP, translationCoeffs.kI, translationCoeffs.kD);
        yController = new PIDController(translationCoeffs.kP, translationCoeffs.kI, translationCoeffs.kD);
        rController = new PIDController(rotationCoeffs.kP, rotationCoeffs.kI, rotationCoeffs.kD);

        withName("Holonomic Vector Control");
    }

    /**
     * Constructor for HolonomicVectorDriveTask using a default Mecanum binding.
     * Left stick controls translation, right stick controls rotation.
     *
     * @param driver              The gamepad to use for driving
     * @param mecanumDrive        The MecanumDrive to use
     * @param fieldCentricEnabled A BooleanSupplier that returns whether field centric drive is enabled
     */
    public HolonomicVectorDriveTask(Gamepad driver, @NotNull MecanumDrive mecanumDrive, BooleanSupplier fieldCentricEnabled) {
        this(() -> driver.left_stick_x, () -> driver.left_stick_y, () -> driver.right_stick_x, mecanumDrive, fieldCentricEnabled);
    }

    @Override
    protected void periodic() {
        Pose2d current = drive.getPoseEstimate();
        // Create a new pose based off the user input, which will be the offset from the current pose.
        // Must rotate by 90 degrees (y, -x), then flip y as it is inverted. Rotation must also be inverted as it
        // must be positive anticlockwise.
        double userX = -y.get();
        double userY = -x.get();
        double userR = -r.get();

        if (fieldCentricEnabled.getAsBoolean()) {
            // Field-centric inputs that will be rotated before any processing
            double botHeading = drive.getExternalHeading();
            double tempX = userX;
            userX = userY * Math.sin(botHeading) + userX * Math.cos(botHeading);
            userY = userY * Math.cos(botHeading) - tempX * Math.sin(botHeading);
        }

        // Rising edge detections for pose locking
        if (userX == 0 && xLock == 0) {
            xLock = current.getX();
            // We also reset the controllers as the integral term may be incorrect due to a new target
            xController.reset();
        } else if (userX != 0) {
            xLock = 0;
        }
        if (userY == 0 && yLock == 0) {
            yLock = current.getY();
            yController.reset();
        } else if (userY != 0) {
            yLock = 0;
        }
        if (userR == 0 && rLock == 0) {
            rLock = current.getHeading();
            rController.reset();
        } else if (userR != 0) {
            rLock = 0;
        }

        // Calculate error from current pose to target pose.
        // If we are not locked, the error will be 0.
        double xLockedError = xLock == 0 ? 0 : xLock - current.getX();
        double yLockedError = yLock == 0 ? 0 : yLock - current.getY();
        double rLockedError = rLock == 0 ? 0 : rLock - current.getHeading();

        // Rotate error to robot's coordinate frame
        double cos = Math.cos(current.getHeading());
        double sin = Math.sin(current.getHeading());
        double twistedXError = xLockedError * cos + yLockedError * sin;
        double twistedYError = -xLockedError * sin + yLockedError * cos;

        // Wrap to [-pi, pi] and hard lock at boundary to ensure no oscillations
        double angle = Mathf.inputModulus(rLockedError, -Math.PI, Math.PI);
        if (Mathf.isNear(Math.abs(angle), Math.PI, 0.1))
            angle = -Math.PI * Math.signum(rLockedError);

        drive.setDriveSignal(
                new DriveSignal(
                    new Pose2d(
                            xLock != 0 ? -xController.calculate(twistedXError) : userX,
                            yLock != 0 ? -yController.calculate(twistedYError) : userY,
                            rLock != 0 ? -rController.calculate(angle) : userR
                    )
                )
        );
    }

    @Override
    protected void onFinish() {
        drive.stop();
    }
}

