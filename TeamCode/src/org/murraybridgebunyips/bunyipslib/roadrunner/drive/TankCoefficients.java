package org.murraybridgebunyips.bunyipslib.roadrunner.drive;

import com.acmerobotics.roadrunner.control.PIDCoefficients;

/**
 * Constants for RoadRunner tank drive.
 * Reworked to use a builder for multiple robot configurations.
 *
 * @author Lucas Bubner, 2023
 */
public class TankCoefficients {
    public PIDCoefficients AXIAL_PID = new PIDCoefficients(0, 0, 0);
    public PIDCoefficients CROSS_TRACK_PID = new PIDCoefficients(0, 0, 0);
    public PIDCoefficients HEADING_PID = new PIDCoefficients(0, 0, 0);
    public double VX_WEIGHT = 1;
    public double OMEGA_WEIGHT = 1;

    public static class Builder {

        private final TankCoefficients tankCoefficients;

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

        public TankCoefficients build() {
            return tankCoefficients;
        }
    }
}
