package org.murraybridgebunyips.bunyipslib.roadrunner.drive;

import com.acmerobotics.roadrunner.control.PIDCoefficients;

/**
 * Constants for RoadRunner mecanum drive.
 * Reworked to use a builder for multiple robot configurations.
 *
 * @author Lucas Bubner, 2023
 */
public class MecanumCoefficients {
    /**
     * The translational PID coefficients to ensure the robot moves in a straight line.
     */
    public PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(0, 0, 0);
    /**
     * The heading PID coefficients to ensure the robot does not drift while moving.
     */
    public PIDCoefficients HEADING_PID = new PIDCoefficients(0, 0, 0);
    /**
     * The lateral multiplier to ensure the robot moves in a straight line.
     */
    public double LATERAL_MULTIPLIER = 1;
    /**
     * The weight of the x velocity in the drive vector.
     */
    public double VX_WEIGHT = 1;
    /**
     * The weight of the y velocity in the drive vector.
     */
    public double VY_WEIGHT = 1;
    /**
     * The weight of the angular velocity in the drive vector.
     */
    public double OMEGA_WEIGHT = 1;

    /**
     * Utility builder to make constructing the coefficients easier.
     */
    public static class Builder {

        private final MecanumCoefficients mecanumCoefficients;

        /**
         * Begin building the coefficients.
         */
        public Builder() {
            mecanumCoefficients = new MecanumCoefficients();
        }

        public Builder setTranslationalPID(PIDCoefficients translationalPID) {
            mecanumCoefficients.TRANSLATIONAL_PID = translationalPID;
            return this;
        }

        public Builder setHeadingPID(PIDCoefficients headingPID) {
            mecanumCoefficients.HEADING_PID = headingPID;
            return this;
        }

        public Builder setLateralMultiplier(double lateralMultiplier) {
            mecanumCoefficients.LATERAL_MULTIPLIER = lateralMultiplier;
            return this;
        }

        public Builder setVXWeight(double vxWeight) {
            mecanumCoefficients.VX_WEIGHT = vxWeight;
            return this;
        }

        public Builder setVYWeight(double vyWeight) {
            mecanumCoefficients.VY_WEIGHT = vyWeight;
            return this;
        }

        public Builder setOmegaWeight(double omegaWeight) {
            mecanumCoefficients.OMEGA_WEIGHT = omegaWeight;
            return this;
        }

        /**
         * Build the coefficients.
         *
         * @return The coefficients
         */
        public MecanumCoefficients build() {
            return mecanumCoefficients;
        }
    }
}
