package org.murraybridgebunyips.bunyipslib.roadrunner.drive;

import com.qualcomm.robotcore.hardware.PIDFCoefficients;

/**
 * Constants shared between multiple drive types.
 * Reworked to use a builder for multiple robot configurations.
 */
public class DriveConstants {

    /*
     * These are motor constants that should be listed online for your motors.
     */
    /**
     * The number of ticks per revolution of the motor's output shaft.
     */
    public double TICKS_PER_REV = 1;
    /**
     * The maximum RPM of the motor.
     */
    public double MAX_RPM = 1;
    /**
     * Set RUN_USING_ENCODER to true to enable built-in hub velocity control using drive encoders.
     * Set this flag to false if drive encoders are not present and an alternative localization
     * method is in use (e.g., tracking wheels).
     * If using the built-in motor velocity PID, update MOTOR_VELO_PID with the tuned coefficients
     * from DriveVelocityPIDTuner.
     */
    public boolean RUN_USING_ENCODER;
    /**
     * The PIDF coefficients for the motor velocity PID.
     */
    public PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(0, 0, 0,
            getMotorVelocityF(MAX_RPM / 60 * TICKS_PER_REV));
    /*
     * These are physical constants that can be determined from your robot (including the track
     * width; it will be tune empirically later although a rough estimate is important). Users are
     * free to chose whichever linear distance unit they would like so long as it is consistently
     * used. The default values were selected with inches in mind. Road runner uses radians for
     * angular distances although most angular parameters are wrapped in Math.toRadians() for
     * convenience. Make sure to exclude any gear ratio included in MOTOR_CONFIG from GEAR_RATIO.
     */
    /**
     * The radius of the wheel in inches.
     */
    public double WHEEL_RADIUS = 2; // in
    /**
     * The gear ratio of the motor. The output (wheel) speed / input (motor) speed.
     */
    public double GEAR_RATIO = 1; // output (wheel) speed / input (motor) speed
    /**
     * The track width is the distance between the left and right wheels on the robot (inches).
     */
    public double TRACK_WIDTH = 1; // in
    /*
     * These are the feedforward parameters used to model the drive motor behavior. If you are using
     * the built-in velocity PID, *these values are fine as is*. However, if you do not have drive
     * motor encoders or have elected not to use them for velocity control, these values should be
     * empirically tuned.
     */
    /**
     * Feedforward kV gain for the velocity PID.
     */
    public double kV = 1.0 / rpmToVelocity(MAX_RPM);
    /**
     * Feedforward kA gain for the velocity PID.
     */
    public double kA;
    /**
     * Feedforward kStatic gain for the velocity PID.
     */
    public double kStatic;
    /*
     * These values are used to generate the trajectories for you robot. To ensure proper operation,
     * the constraints should never exceed ~80% of the robot's actual capabilities. While Road
     * Runner is designed to enable faster autonomous motion, it is a good idea for testing to start
     * small and gradually increase them later after everything is working. All distance units are
     * inches.
     */
    /**
     * The maximum velocity of the robot in inches per second.
     */
    public double MAX_VEL = 30;
    /**
     * The maximum acceleration of the robot in inches per second squared.
     */
    public double MAX_ACCEL = 30;
    /**
     * The maximum angular velocity of the robot in radians per second.
     */
    public double MAX_ANG_VEL = Math.toRadians(60);
    /**
     * The maximum angular acceleration of the robot in radians per second squared.
     */
    public double MAX_ANG_ACCEL = Math.toRadians(60);

    /**
     * Get the motor velocity feedforward gain.
     *
     * @param ticksPerSecond the number of ticks per second
     * @return the motor velocity feedforward gain
     */
    public static double getMotorVelocityF(double ticksPerSecond) {
        // see https://docs.google.com/document/d/1tyWrXDfMidwYyP_5H4mZyVgaEswhOC35gvdmP-V-5hA/edit#heading=h.61g9ixenznbx
        return 32767 / ticksPerSecond;
    }

    /**
     * Convert encoder ticks to wheel distance in inches.
     *
     * @param ticks the encoder ticks
     * @return the wheel distance in inches
     */
    public double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    /**
     * Convert motor RPM to wheel velocity in inches per second.
     *
     * @param rpm the motor RPM
     * @return the wheel velocity in inches per second
     */
    public double rpmToVelocity(double rpm) {
        return rpm * GEAR_RATIO * 2 * Math.PI * WHEEL_RADIUS / 60.0;
    }

    /**
     * Builder class to assist in making DriveConstants.
     */
    public static class Builder {

        private final DriveConstants driveConstants;

        /**
         * Begin building a new DriveConstants object.
         */
        public Builder() {
            driveConstants = new DriveConstants();
        }

        public Builder setTicksPerRev(double ticksPerRev) {
            driveConstants.TICKS_PER_REV = ticksPerRev;
            return this;
        }

        public Builder setMaxRPM(double maxRPM) {
            driveConstants.MAX_RPM = maxRPM;
            return this;
        }

        public Builder setRunUsingEncoder(boolean runUsingEncoder) {
            driveConstants.RUN_USING_ENCODER = runUsingEncoder;
            return this;
        }

        public Builder setMotorVeloPID(PIDFCoefficients motorVeloPID) {
            driveConstants.MOTOR_VELO_PID = motorVeloPID;
            return this;
        }

        public Builder setWheelRadius(double wheelRadius) {
            driveConstants.WHEEL_RADIUS = wheelRadius;
            return this;
        }

        public Builder setGearRatio(double gearRatio) {
            driveConstants.GEAR_RATIO = gearRatio;
            return this;
        }

        public Builder setTrackWidth(double trackWidth) {
            driveConstants.TRACK_WIDTH = trackWidth;
            return this;
        }

        public Builder setKV(double kV) {
            driveConstants.kV = kV;
            return this;
        }

        public Builder setKA(double kA) {
            driveConstants.kA = kA;
            return this;
        }

        public Builder setKStatic(double kStatic) {
            driveConstants.kStatic = kStatic;
            return this;
        }

        public Builder setMaxVel(double maxVel) {
            driveConstants.MAX_VEL = maxVel;
            return this;
        }

        public Builder setMaxAccel(double maxAccel) {
            driveConstants.MAX_ACCEL = maxAccel;
            return this;
        }

        public Builder setMaxAngVel(double maxAngVel) {
            driveConstants.MAX_ANG_VEL = maxAngVel;
            return this;
        }

        public Builder setMaxAngAccel(double maxAngAccel) {
            driveConstants.MAX_ANG_ACCEL = maxAngAccel;
            return this;
        }

        /**
         * Finalise your configuration.
         *
         * @return the DriveConstants object
         */
        public DriveConstants build() {
            return driveConstants;
        }
    }
}
