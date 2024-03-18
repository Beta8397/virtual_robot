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

        public Builder setAxialPID(PIDCoefficients axialPID) {
            tankCoefficients.AXIAL_PID = axialPID;
            return this;
        }

        public Builder setCrossTrackPID(PIDCoefficients crossTrackPID) {
            tankCoefficients.CROSS_TRACK_PID = crossTrackPID;
            return this;
        }

        public Builder setHeadingPID(PIDCoefficients headingPID) {
            tankCoefficients.HEADING_PID = headingPID;
            return this;
        }

        public Builder setVXWeight(double vxWeight) {
            tankCoefficients.VX_WEIGHT = vxWeight;
            return this;
        }

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
