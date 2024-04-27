package org.murraybridgebunyips.bunyipslib.roadrunner.drive;

import com.acmerobotics.roadrunner.control.PIDCoefficients;

/**
 * Constants for RoadRunner tank drive.
 * Reworked to use a builder for multiple robot configurations.
 *
 * @author Lucas Bubner, 2023
 */
public class TankCoefficients {
    /**
     * PID coefficients for the axial (forward) control.
     */
    public PIDCoefficients AXIAL_PID = new PIDCoefficients(0, 0, 0);
    /**
     * PID coefficients for the cross-track (strafe) control.
     */
    public PIDCoefficients CROSS_TRACK_PID = new PIDCoefficients(0, 0, 0);
    /**
     * PID coefficients for the heading control.
     */
    public PIDCoefficients HEADING_PID = new PIDCoefficients(0, 0, 0);
    /**
     * Weight for the velocity error in the axial PID.
     */
    public double VX_WEIGHT = 1;
    /**
     * Weight for the velocity error in the heading PID.
     */
    public double OMEGA_WEIGHT = 1;

    /**
     * Utility builder for creating new coefficients.
     */
    public static class Builder {

        private final TankCoefficients tankCoefficients;

        /**
         * Start building.
         */
        public Builder() {
            tankCoefficients = new TankCoefficients();
        }

        /**
         * Set the axial (forward) PID coefficients.
         *
         * @param axialPID The axial PID coefficients
         * @return The builder
         */
        public Builder setAxialPID(PIDCoefficients axialPID) {
            tankCoefficients.AXIAL_PID = axialPID;
            return this;
        }

        /**
         * Set the cross-track (strafe) PID coefficients.
         *
         * @param crossTrackPID The cross-track PID coefficients
         * @return The builder
         */
        public Builder setCrossTrackPID(PIDCoefficients crossTrackPID) {
            tankCoefficients.CROSS_TRACK_PID = crossTrackPID;
            return this;
        }

        /**
         * Set the heading PID coefficients.
         *
         * @param headingPID The heading PID coefficients
         * @return The builder
         */
        public Builder setHeadingPID(PIDCoefficients headingPID) {
            tankCoefficients.HEADING_PID = headingPID;
            return this;
        }

        /**
         * Set the weight for the velocity error in the axial PID.
         *
         * @param vxWeight The weight for the velocity error in the axial PID
         * @return The builder
         */
        public Builder setVXWeight(double vxWeight) {
            tankCoefficients.VX_WEIGHT = vxWeight;
            return this;
        }

        /**
         * Set the weight for the velocity error in the heading PID.
         *
         * @param omegaWeight The weight for the velocity error in the heading PID
         * @return The builder
         */
        public Builder setOmegaWeight(double omegaWeight) {
            tankCoefficients.OMEGA_WEIGHT = omegaWeight;
            return this;
        }

        /**
         * Build the coefficients.
         *
         * @return The coefficients
         */
        public TankCoefficients build() {
            return tankCoefficients;
        }
    }
}
