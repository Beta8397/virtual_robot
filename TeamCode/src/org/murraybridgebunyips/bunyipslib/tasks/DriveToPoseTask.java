package org.murraybridgebunyips.bunyipslib.tasks;

import static org.murraybridgebunyips.bunyipslib.external.units.Units.Centimeters;
import static org.murraybridgebunyips.bunyipslib.external.units.Units.Degrees;
import static org.murraybridgebunyips.bunyipslib.external.units.Units.Inches;
import static org.murraybridgebunyips.bunyipslib.external.units.Units.Radians;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.murraybridgebunyips.bunyipslib.BunyipsSubsystem;
import org.murraybridgebunyips.bunyipslib.external.Mathf;
import org.murraybridgebunyips.bunyipslib.external.pid.PIDFController;
import org.murraybridgebunyips.bunyipslib.external.units.Angle;
import org.murraybridgebunyips.bunyipslib.external.units.Distance;
import org.murraybridgebunyips.bunyipslib.external.units.Measure;
import org.murraybridgebunyips.bunyipslib.external.units.Time;
import org.murraybridgebunyips.bunyipslib.roadrunner.drive.RoadRunnerDrive;
import org.murraybridgebunyips.bunyipslib.tasks.bases.Task;

/**
 * Drive to a pose using RoadRunner.
 * This is useful for pose alignment based on error, rather than using trajectories for planned motion.
 * Designed and tested for Mecanum drivebases as RoadRunner was designed around them.
 *
 * @author Lucas Bubner, 2024
 */
public class DriveToPoseTask extends Task {
    private final RoadRunnerDrive drive;
    private final Pose2d targetPose;
    private final PIDFController forwardController;
    private final PIDFController strafeController;
    private final PIDFController headingController;

    private double MAX_FORWARD_SPEED = 1.0;
    private double MAX_STRAFE_SPEED = 1.0;
    private double MAX_ROTATION_SPEED = 1.0;

    private Measure<Angle> headingTolerance = Degrees.of(2);
    private Measure<Distance> vectorTolerance = Centimeters.of(5);

    /**
     * Run the Drive To Pose Task on a drive subsystem.
     *
     * @param timeout           The maximum time the task can run for.
     * @param driveSubsystem    The drive subsystem to run the task on.
     * @param targetPose        The target pose to drive to.
     * @param forwardController The PID controller for x.
     * @param strafeController  The PID controller for y.
     * @param headingController The PID controller for heading.
     */
    public DriveToPoseTask(@NonNull Measure<Time> timeout, @NonNull BunyipsSubsystem driveSubsystem,
                           Pose2d targetPose, PIDFController forwardController, PIDFController strafeController, PIDFController headingController) {
        super(timeout, driveSubsystem, true);
        if (!(driveSubsystem instanceof RoadRunnerDrive))
            throw new IllegalArgumentException("DriveToPoseTask requires a RoadRunnerDrive subsystem");
        drive = (RoadRunnerDrive) driveSubsystem;
        this.targetPose = targetPose;
        this.forwardController = forwardController;
        this.strafeController = strafeController;
        this.headingController = headingController;
        withName("Drive To Pose: " + targetPose.toString());
    }

    /**
     * Set the tolerances for the task.
     *
     * @param heading     The tolerance for heading.
     * @param translation The tolerance for the translation vector.
     * @return this
     */
    public DriveToPoseTask withTolerances(Measure<Angle> heading, Measure<Distance> translation) {
        headingTolerance = heading;
        vectorTolerance = translation;
        return this;
    }

    /**
     * Set the maximum forward (x) speed (motor power) that the robot can move at.
     *
     * @param speed The maximum forward speed.
     * @return this
     */
    public DriveToPoseTask withMaxForwardSpeed(double speed) {
        MAX_FORWARD_SPEED = speed;
        return this;
    }

    /**
     * Set the maximum strafe (y) speed (motor power) that the robot can move at.
     *
     * @param speed The maximum strafe speed.
     * @return this
     */
    public DriveToPoseTask withMaxStrafeSpeed(double speed) {
        MAX_STRAFE_SPEED = speed;
        return this;
    }

    /**
     * Set the maximum rotation speed (motor power) that the robot can move at.
     *
     * @param speed The maximum rotation speed.
     * @return this
     */
    public DriveToPoseTask withMaxRotationSpeed(double speed) {
        MAX_ROTATION_SPEED = speed;
        return this;
    }

    @Override
    protected void init() {
        drive.cancelTrajectory();
        drive.stop();
    }

    public boolean isVectorNear() {
        return Mathf.isNear(0, drive.getPoseEstimate().vec().distTo(targetPose.vec()), vectorTolerance.in(Inches));
    }

    public boolean isHeadingNear() {
        return Mathf.isNear(targetPose.getHeading(), drive.getPoseEstimate().getHeading(), headingTolerance.in(Radians));
    }

    @Override
    protected void periodic() {
        Pose2d estimatedPose = drive.getPoseEstimate();
        Pose2d error = targetPose.minus(estimatedPose);

        // Twist the error vector to be relative to the robot's heading, as rotations of the robot are not
        // accounted for in the RoadRunner pose estimate
        double cos = Math.cos(estimatedPose.getHeading());
        double sin = Math.sin(estimatedPose.getHeading());

        // Transform error vector to robot's coordinate frame
        double twistedXError = error.getX() * cos + error.getY() * sin;
        double twistedYError = -error.getX() * sin + error.getY() * cos;

        // Wrap target angle between -pi and pi for optimal turns
        double angle = Mathf.inputModulus(error.getHeading(), -Math.PI, Math.PI);
        // When the angle is near the modulus boundary, lock towards a definitive full rotation to avoid oscillations
        if (Mathf.isNear(Math.abs(angle), Math.PI, 0.1))
            angle = -Math.PI * Math.signum(error.getHeading());

        // Apply PID and twist
        double forwardPower = -forwardController.calculate(twistedXError);
        double strafePower = -strafeController.calculate(twistedYError);
        double headingPower = -headingController.calculate(angle);

        drive.setWeightedDrivePower(
                new Pose2d(
                        Mathf.clamp(forwardPower, -MAX_FORWARD_SPEED, MAX_FORWARD_SPEED),
                        Mathf.clamp(strafePower, -MAX_STRAFE_SPEED, MAX_STRAFE_SPEED),
                        Mathf.clamp(headingPower, -MAX_ROTATION_SPEED, MAX_ROTATION_SPEED)
                )
        );
    }

    @Override
    protected void onFinish() {
        drive.stop();
    }

    @Override
    protected boolean isTaskFinished() {
        return isVectorNear() && isHeadingNear();
    }
}
