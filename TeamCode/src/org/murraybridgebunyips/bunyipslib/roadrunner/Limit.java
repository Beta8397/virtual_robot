package org.murraybridgebunyips.bunyipslib.roadrunner;

import static org.murraybridgebunyips.bunyipslib.external.units.Units.InchesPerSecond;
import static org.murraybridgebunyips.bunyipslib.external.units.Units.RadiansPerSecond;
import static org.murraybridgebunyips.bunyipslib.external.units.Units.Second;

import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;

import org.murraybridgebunyips.bunyipslib.external.units.Angle;
import org.murraybridgebunyips.bunyipslib.external.units.Distance;
import org.murraybridgebunyips.bunyipslib.external.units.Measure;
import org.murraybridgebunyips.bunyipslib.external.units.Velocity;
import org.murraybridgebunyips.bunyipslib.roadrunner.drive.RoadRunnerDrive;

/**
 * Utility methods for RoadRunner velocity and acceleration constraints.
 *
 * @author Lucas Bubner, 2024
 */
public final class Limit {
    private Limit() {
    }

    /**
     * Returns a TrajectoryVelocityConstraint based on the given DriveConstants and translational velocity.
     *
     * @param translationalVelocity The translational velocity to use.
     * @param drive                 The DriveConstants to use.
     * @return A TrajectoryVelocityConstraint based on the given parameters.
     */
    public static TrajectoryVelocityConstraint ofVelocity(Measure<Velocity<Distance>> translationalVelocity, RoadRunnerDrive drive) {
        return drive.getVelocityConstraint(translationalVelocity.in(InchesPerSecond), drive.getConstants().MAX_ANG_VEL, drive.getConstants().TRACK_WIDTH);
    }

    /**
     * Returns a TrajectoryVelocityConstraint based on the given DriveConstants and angular velocity.
     *
     * @param angularVelocity The angular velocity to use.
     * @param drive           The DriveConstants to use.
     * @return A TrajectoryVelocityConstraint based on the given parameters.
     */
    public static TrajectoryVelocityConstraint ofAngularVelocity(Measure<Velocity<Angle>> angularVelocity, RoadRunnerDrive drive) {
        return drive.getVelocityConstraint(drive.getConstants().MAX_VEL, angularVelocity.in(RadiansPerSecond), drive.getConstants().TRACK_WIDTH);
    }

    /**
     * Returns a TrajectoryVelocityConstraint based on the given DriveConstants, translational velocity, and angular velocity.
     *
     * @param translationalVelocity The translational velocity to use.
     * @param angularVelocity       The angular velocity to use.
     * @param drive                 The DriveConstants to use.
     * @return A TrajectoryVelocityConstraint based on the given parameters.
     */
    public static TrajectoryVelocityConstraint ofVelocities(Measure<Velocity<Distance>> translationalVelocity, Measure<Velocity<Angle>> angularVelocity, RoadRunnerDrive drive) {
        return drive.getVelocityConstraint(translationalVelocity.in(InchesPerSecond), angularVelocity.in(RadiansPerSecond), drive.getConstants().TRACK_WIDTH);
    }

    /**
     * Returns a TrajectoryAccelerationConstraint based on the given DriveConstants and translational acceleration.
     *
     * @param translationalAcceleration The translational acceleration to use.
     * @param drive                     The DriveConstants to use.
     * @return A TrajectoryAccelerationConstraint based on the given parameters.
     */
    public static TrajectoryAccelerationConstraint ofAcceleration(Measure<Velocity<Velocity<Distance>>> translationalAcceleration, RoadRunnerDrive drive) {
        return drive.getAccelerationConstraint(translationalAcceleration.in(InchesPerSecond.per(Second)));
    }
}
