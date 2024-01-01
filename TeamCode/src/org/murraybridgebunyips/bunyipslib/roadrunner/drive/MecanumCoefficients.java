package org.murraybridgebunyips.bunyipslib.roadrunner.drive;

import com.acmerobotics.roadrunner.control.PIDCoefficients;

/**
 * Constants for RoadRunner mecanum drive.
 * Reworked to use a builder for multiple robot configurations.
 *
 * @author Lucas Bubner, 2023
 */
public class MecanumCoefficients {
    public PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(0, 0, 0);
    public PIDCoefficients HEADING_PID = new PIDCoefficients(0, 0, 0);
    public double LATERAL_MULTIPLIER = 1;
    public double VX_WEIGHT = 1;
    public double VY_WEIGHT = 1;
    public double OMEGA_WEIGHT = 1;

    public static class Builder {

        private final MecanumCoefficients mecanumCoefficients;

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

        public MecanumCoefficients build() {
            return mecanumCoefficients;
        }
    }
}
