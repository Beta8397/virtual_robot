package org.murraybridgebunyips.bunyipslib.drive;

import static org.murraybridgebunyips.bunyipslib.external.units.Units.Degrees;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.murraybridgebunyips.bunyipslib.Direction;

/**
 * Variant of the Cartesian MecanumDrive that uses field-centric controls, accounting for robot heading.
 * <p>
 * This system is deprecated by RoadRunner, but remains for legacy/non-RoadRunner purposes.
 * Coordinates used in Cartesian drives include a Cartesian speed mapping, where X is the horizontal axis, and
 * Y is the forward axis. This is rotated differently within RoadRunner-derived drive bases.
 * <p>
 * Note: CFCMD no longer relies on an IMUOp subsystem, and can simply be used with a standard IMU from your config.
 *
 * @author Lucas Bubner, 2023
 * @see MecanumDrive
 */
public class CartesianFieldCentricMecanumDrive extends CartesianMecanumDrive {
    private IMU imu;
    private double offsetDeg;

    /**
     * Constructs a new CartesianFieldCentricMecanumDrive.
     *
     * @param frontLeft                 the front left motor
     * @param frontRight                the front right motor
     * @param backLeft                  the back left motor
     * @param backRight                 the back right motor
     * @param imu                       the IMU to use
     * @param invalidatePreviousHeading whether to invalidate the previous heading of the IMU
     * @param startingDirection         the starting direction of the robot relative to the field-centric origin
     */
    public CartesianFieldCentricMecanumDrive(DcMotor frontLeft, DcMotor frontRight, DcMotor backLeft, DcMotor backRight, IMU imu, boolean invalidatePreviousHeading, Direction startingDirection) {
        super(frontLeft, frontRight, backLeft, backRight);
        if (!assertParamsNotNull(imu)) return;

        this.imu = imu;

        // Invalidate any previous readings
        if (invalidatePreviousHeading)
            imu.resetYaw();

        // Current vector will be the robot's starting vector, must offset the IMU to align straight
        offsetDeg = startingDirection.getAngle().in(Degrees);
    }

    /**
     * Set a speed at which the Mecanum drive assembly should move.
     * This method will also account for the robot's current heading and adjust the vectors
     * accordingly.
     *
     * @param x The speed at which the robot should move in the x direction relative to the field.
     *          Positive is right, negative is left.
     *          Range: -1.0 to 1.0
     * @param y The speed at which the robot should move in the y direction relative to the field.
     *          Positive is forward, negative is backward.
     *          Range: -1.0 to 1.0
     * @param r The speed at which the robot will rotate.
     *          Positive is clockwise, negative is anti-clockwise.
     *          Range: -1.0 to 1.0
     */
    @Override
    public CartesianFieldCentricMecanumDrive setSpeedXYR(double x, double y, double r) {
        double botHeading = Math.toRadians(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) + offsetDeg);

        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        super.setSpeedXYR(rotX, rotY, r);

        return this;
    }
}
