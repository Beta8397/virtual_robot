package org.murraybridgebunyips.bunyipslib.subsystems;

import static org.murraybridgebunyips.bunyipslib.external.units.Units.Degrees;
import static org.murraybridgebunyips.bunyipslib.external.units.Units.DegreesPerSecond;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.murraybridgebunyips.bunyipslib.BunyipsSubsystem;
import org.murraybridgebunyips.bunyipslib.external.Mathf;
import org.murraybridgebunyips.bunyipslib.external.units.Angle;
import org.murraybridgebunyips.bunyipslib.external.units.Measure;
import org.murraybridgebunyips.bunyipslib.external.units.Velocity;
import org.murraybridgebunyips.bunyipslib.tasks.bases.Task;

/**
 * IMU utility class that will wrap an Inertial Measurement Unit to provide data and automatic updates
 * of the IMU angles as part of a subsystem. This subsystem has no meaningful {@link Task} to schedule.
 * <p>
 * This class, alike the old IMUOp also supports different readings of IMU measurement, such as an unrestricted domain
 * on heading reads, while also being able to provide IMU units in terms of WPIUnits.
 * <p>
 * Angles from this class are intrinsic. Read more in the {@link YawPitchRollAngles} and {@link Orientation} classes.
 *
 * @author Lucas Bubner, 2024
 */
public class IMUOp extends BunyipsSubsystem {
    private final IMU imu;

    /**
     * Last read yaw (heading) of the IMU.
     * The domain of this value is controlled by the currently set by the {@link YawDomain} (see {@link #setYawDomain}).
     */
    @NonNull
    public volatile Measure<Angle> yaw = Degrees.zero();
    /**
     * Last read pitch of the IMU.
     */
    @NonNull
    public volatile Measure<Angle> pitch = Degrees.zero();
    /**
     * Last read roll of the IMU.
     */
    @NonNull
    public volatile Measure<Angle> roll = Degrees.zero();
    /**
     * Last read yaw velocity of the IMU.
     */
    @NonNull
    public volatile Measure<Velocity<Angle>> yawVel = DegreesPerSecond.zero();
    /**
     * Last read pitch velocity of the IMU.
     */
    @NonNull
    public volatile Measure<Velocity<Angle>> pitchVel = DegreesPerSecond.zero();
    /**
     * Last read roll velocity of the IMU.
     */
    @NonNull
    public volatile Measure<Velocity<Angle>> rollVel = DegreesPerSecond.zero();
    /**
     * Last acquisition of data from the IMU by IMUOp. Nullable if this subsystem hasn't updated. Unit is in
     * nanoseconds as returned by {@link System#nanoTime()}.
     */
    @Nullable
    public volatile Long lastAcquisitionTimeNanos = null;

    @NonNull
    private YawDomain domain = YawDomain.SIGNED;
    private Measure<Angle> yawOffset = Degrees.zero();
    private double angleSumDeg;
    private double lastYawDeg;

    /**
     * Wrap an IMU to use in IMUOp.
     *
     * @param imu the imu to wrap
     */
    public IMUOp(IMU imu) {
        this.imu = imu;
    }

    /**
     * Resets the robot's yaw angle to 0. After calling this method, the reported orientation will
     * be relative to the robot's position when this method was called, as if the robot was perfectly
     * level right then. That is to say, the pitch and yaw will be ignored when this method is
     * called.
     * <p>
     * Unlike yaw, pitch and roll are always relative to gravity, and never need to be reset.
     */
    public void resetYaw() {
        angleSumDeg = 0;
        lastYawDeg = 0;
        imu.resetYaw();
    }

    /**
     * Gets the current yaw domain, which is the domain this IMU is reporting {@link #yaw} at.
     *
     * @return the yaw domain that can be used to view the current domain and convert between
     */
    public YawDomain getYawDomain() {
        return domain;
    }

    /**
     * Sets the domain range of what the {@link #yaw} field will return. By default, this is set to the expected
     * behaviour of [-180, 180) degrees, but can be adjusted here to one of the {@link YawDomain} options.
     *
     * @param newDomain the new domain of the {@link #yaw} property
     */
    public void setYawDomain(YawDomain newDomain) {
        if (newDomain == null) return;
        domain = newDomain;
    }

    /**
     * @return the current yaw offset as respected by this class
     */
    public Measure<Angle> getYawOffset() {
        return yawOffset;
    }

    /**
     * Set an offset of the yaw value that will apply for all future yaw reads in this class.
     *
     * @param yawOffset the yaw offset that will be added to the actual yaw
     */
    public void setYawOffset(Measure<Angle> yawOffset) {
        if (yawOffset == null) return;
        this.yawOffset = yaw;
    }

    @Override
    protected void periodic() {
        // Returned units from IMU will be in degrees and degrees/sec
        // Since these fields are internally stored as degrees, it is more accurate we don't convert between and
        // simply use the units we were provided to construct the member fields.
        YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
        AngularVelocity angleVels = imu.getRobotAngularVelocity(AngleUnit.DEGREES);
        lastAcquisitionTimeNanos = System.nanoTime();

        double yawDeg = angles.getYaw(AngleUnit.DEGREES);
        double yawDelta = yawDeg - lastYawDeg;
        if (Math.abs(yawDelta) >= 180) {
            // IMU returns angles in signed form, need to undo this as we'll clamp this value ourselves
            yawDelta = -Math.signum(yawDelta) * (360 - Math.abs(yawDelta));
        }
        lastYawDeg = yawDeg;

        angleSumDeg += yawDelta;
        Measure<Angle> totalYaw = Degrees.of(angleSumDeg);

        if (domain == YawDomain.SIGNED) {
            yaw = Mathf.angleModulus(totalYaw);
        } else if (domain == YawDomain.UNSIGNED) {
            yaw = Mathf.normaliseAngle(totalYaw);
        } else {
            // Unrestricted domain
            yaw = totalYaw;
        }

        // These fields are also bound by the [-180, 180) degree domain but can be converted with Mathf utilities.
        // IMUOp provides a built in utility for the yaw, as it is a common use case and usually you wouldn't need
        // to use these fields in a different domain.
        pitch = Degrees.of(angles.getPitch(AngleUnit.DEGREES));
        roll = Degrees.of(angles.getRoll(AngleUnit.DEGREES));

        yawVel = DegreesPerSecond.of(angleVels.zRotationRate);
        pitchVel = DegreesPerSecond.of(angleVels.xRotationRate);
        rollVel = DegreesPerSecond.of(angleVels.yRotationRate);
    }

    /**
     * The various modes that the {@link #yaw} field can represent the current robot yaw as.
     * If you wish to convert between these domains, see the utilities available in {@link Mathf}.
     */
    public enum YawDomain {
        /**
         * Default behaviour. Angle is wrapped between [-180, 180) degrees, or [-π, π) radians.
         */
        SIGNED,
        /**
         * Angle is wrapped between [0, 360) degrees, or [0, 2π) radians.
         */
        UNSIGNED,
        /**
         * Angle is unrestricted between (-∞, ∞) units.
         */
        UNRESTRICTED
    }
}
