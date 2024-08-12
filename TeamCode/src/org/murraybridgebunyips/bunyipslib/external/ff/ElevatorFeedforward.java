/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.murraybridgebunyips.bunyipslib.external.ff;

import org.murraybridgebunyips.bunyipslib.external.SystemController;

/**
 * A helper class that computes feedforward outputs for a simple elevator (modeled as a motor
 * acting against the force of gravity). Units of this class are determined by the inputs to the gains.
 * <a href="https://github.com/FTCLib/FTCLib/blob/1c8995d09413b406e0f4aff238ea4edc2bb860c4/core/src/main/java/com/arcrobotics/ftclib/controller/wpilibcontroller/ElevatorFeedforward.java">Source</a>
 */
public class ElevatorFeedforward implements SystemController {
    private double kS;
    private double kG;
    private double kV;
    private double kA;

    /**
     * Creates a new ElevatorFeedforward with the specified gains.  Units of the gain values
     * will dictate units of the computed feedforward.
     *
     * @param kS The static gain.
     * @param kG The gravity gain.
     * @param kV The velocity gain.
     * @param kA The acceleration gain.
     */
    public ElevatorFeedforward(double kS, double kG, double kV, double kA) {
        this.kS = kS;
        this.kG = kG;
        this.kV = kV;
        this.kA = kA;
    }

    /**
     * Creates a new ElevatorFeedforward with the specified gains.  Acceleration gain is
     * defaulted to zero.  Units of the gain values will dictate units of the computed feedforward.
     *
     * @param kS The static gain.
     * @param kG The gravity gain.
     * @param kV The velocity gain.
     */
    public ElevatorFeedforward(double kS, double kG, double kV) {
        this(kS, kG, kV, 0);
    }

    /**
     * Static gain.
     */
    public double getS() {
        return kS;
    }

    /**
     * Gravity gain.
     */
    public double getG() {
        return kG;
    }

    /**
     * Velocity gain.
     */
    public double getV() {
        return kV;
    }

    /**
     * Acceleration gain.
     */
    public double getA() {
        return kA;
    }

    /**
     * Calculates the feedforward from the gains and setpoints.
     *
     * @param velocity     The velocity setpoint.
     * @param acceleration The acceleration setpoint.
     * @return The computed feedforward.
     */
    @Override
    public double calculate(double velocity, double acceleration) {
        return kS * Math.signum(velocity) + kG + kV * velocity + kA * acceleration;
    }

    @Override
    public void reset() {
        // no-op
    }

    /**
     * Calculates the feedforward from the gains and velocity setpoint (acceleration is assumed to
     * be zero).
     *
     * @param velocity The velocity setpoint.
     * @return The computed feedforward.
     */
    public double calculate(double velocity) {
        return calculate(velocity, 0);
    }

    // Rearranging the main equation from the calculate() method yields the
    // formulas for the methods below:

    /**
     * Calculates the maximum achievable velocity given a maximum voltage supply
     * and an acceleration.  Useful for ensuring that velocity and
     * acceleration constraints for a trapezoidal profile are simultaneously
     * achievable - enter the acceleration constraint, and this will give you
     * a simultaneously-achievable velocity constraint.
     *
     * @param maxVoltage   The maximum voltage that can be supplied to the elevator.
     * @param acceleration The acceleration of the elevator.
     * @return The maximum possible velocity at the given acceleration.
     */
    public double maxAchievableVelocity(double maxVoltage, double acceleration) {
        // Assume max velocity is positive
        return (maxVoltage - kS - kG - acceleration * kA) / kV;
    }

    /**
     * Calculates the minimum achievable velocity given a maximum voltage supply
     * and an acceleration.  Useful for ensuring that velocity and
     * acceleration constraints for a trapezoidal profile are simultaneously
     * achievable - enter the acceleration constraint, and this will give you
     * a simultaneously-achievable velocity constraint.
     *
     * @param maxVoltage   The maximum voltage that can be supplied to the elevator.
     * @param acceleration The acceleration of the elevator.
     * @return The minimum possible velocity at the given acceleration.
     */
    public double minAchievableVelocity(double maxVoltage, double acceleration) {
        // Assume min velocity is negative, ks flips sign
        return (-maxVoltage + kS - kG - acceleration * kA) / kV;
    }

    /**
     * Calculates the maximum achievable acceleration given a maximum voltage
     * supply and a velocity. Useful for ensuring that velocity and
     * acceleration constraints for a trapezoidal profile are simultaneously
     * achievable - enter the velocity constraint, and this will give you
     * a simultaneously-achievable acceleration constraint.
     *
     * @param maxVoltage The maximum voltage that can be supplied to the elevator.
     * @param velocity   The velocity of the elevator.
     * @return The maximum possible acceleration at the given velocity.
     */
    public double maxAchievableAcceleration(double maxVoltage, double velocity) {
        return (maxVoltage - kS * Math.signum(velocity) - kG - velocity * kV) / kA;
    }

    /**
     * Calculates the minimum achievable acceleration given a maximum voltage
     * supply and a velocity. Useful for ensuring that velocity and
     * acceleration constraints for a trapezoidal profile are simultaneously
     * achievable - enter the velocity constraint, and this will give you
     * a simultaneously-achievable acceleration constraint.
     *
     * @param maxVoltage The maximum voltage that can be supplied to the elevator.
     * @param velocity   The velocity of the elevator.
     * @return The minimum possible acceleration at the given velocity.
     */
    public double minAchievableAcceleration(double maxVoltage, double velocity) {
        return maxAchievableAcceleration(-maxVoltage, velocity);
    }

    @Override
    public void setCoefficients(double... coeffs) {
        if (coeffs.length != 4) {
            throw new IllegalArgumentException("expected 4 coefficients, got " + coeffs.length);
        }
        kS = coeffs[0];
        kG = coeffs[1];
        kV = coeffs[2];
        kA = coeffs[3];
    }
}