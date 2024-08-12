/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.murraybridgebunyips.bunyipslib.external.ff;

import static org.murraybridgebunyips.bunyipslib.external.units.Units.Radians;
import static org.murraybridgebunyips.bunyipslib.external.units.Units.RadiansPerSecond;
import static org.murraybridgebunyips.bunyipslib.external.units.Units.RadiansPerSecondPerSecond;
import static org.murraybridgebunyips.bunyipslib.external.units.Units.Volts;

import org.murraybridgebunyips.bunyipslib.external.SystemController;
import org.murraybridgebunyips.bunyipslib.external.units.Angle;
import org.murraybridgebunyips.bunyipslib.external.units.Measure;
import org.murraybridgebunyips.bunyipslib.external.units.Velocity;
import org.murraybridgebunyips.bunyipslib.external.units.Voltage;

/**
 * A helper class that computes feedforward outputs for a simple arm (modeled as a motor
 * acting against the force of gravity on a beam suspended at an angle), adjusted to use WPIUnits for unit assertion.
 * <a href="https://github.com/FTCLib/FTCLib/blob/1c8995d09413b406e0f4aff238ea4edc2bb860c4/core/src/main/java/com/arcrobotics/ftclib/controller/wpilibcontroller/ArmFeedforward.java">Source</a>
 */
public class ArmFeedforward implements SystemController {
    private double kS;
    private double kCos;
    private double kV;
    private double kA;

    /**
     * Creates a new ArmFeedforward with the specified gains.  Units of the gain values
     * will dictate units of the computed feedforward.
     *
     * @param kS   The static gain.
     * @param kCos The gravity gain.
     * @param kV   The velocity gain.
     * @param kA   The acceleration gain.
     */
    public ArmFeedforward(double kS, double kCos, double kV, double kA) {
        this.kS = kS;
        this.kCos = kCos;
        this.kV = kV;
        this.kA = kA;
    }

    /**
     * Creates a new ArmFeedforward with the specified gains.  Acceleration gain is
     * defaulted to zero.  Units of the gain values will dictate units of the computed feedforward.
     *
     * @param kS   The static gain.
     * @param kCos The gravity gain.
     * @param kV   The velocity gain.
     */
    public ArmFeedforward(double kS, double kCos, double kV) {
        this(kS, kCos, kV, 0);
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
    public double getCos() {
        return kCos;
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
     * @param position     The position setpoint.
     * @param velocity     The velocity setpoint.
     * @param acceleration The acceleration setpoint.
     * @return The computed feedforward.
     */
    public double calculate(Measure<Angle> position, Measure<Velocity<Angle>> velocity,
                            Measure<Velocity<Velocity<Angle>>> acceleration) {
        double positionRadians = position.in(Radians);
        double velocityRadPerSec = velocity.in(RadiansPerSecond);
        double accelRadPerSecSquared = acceleration.in(RadiansPerSecondPerSecond);
        return kS * Math.signum(velocityRadPerSec) + kCos * Math.cos(positionRadians)
                + kV * velocityRadPerSec
                + kA * accelRadPerSecSquared;
    }

    /**
     * Calculates the feedforward from the gains and velocity setpoint (acceleration is assumed to
     * be zero).
     *
     * @param position The position setpoint.
     * @param velocity The velocity setpoint.
     * @return The computed feedforward.
     */
    public double calculate(Measure<Angle> position, Measure<Velocity<Angle>> velocity) {
        return calculate(position, velocity, RadiansPerSecondPerSecond.zero());
    }

    // Rearranging the main equation from the calculate() method yields the
    // formulas for the methods below:

    /**
     * Calculates the maximum achievable velocity given a maximum voltage supply,
     * a position, and an acceleration.  Useful for ensuring that velocity and
     * acceleration constraints for a trapezoidal profile are simultaneously
     * achievable - enter the acceleration constraint, and this will give you
     * a simultaneously-achievable velocity constraint.
     *
     * @param maxVoltage   The maximum voltage that can be supplied to the arm.
     * @param angle        The angle of the arm.
     * @param acceleration The acceleration of the arm.
     * @return The maximum possible velocity at the given acceleration and angle.
     */
    public Measure<Velocity<Angle>> maxAchievableVelocity(Measure<Voltage> maxVoltage, Measure<Angle> angle, Measure<Velocity<Velocity<Angle>>> acceleration) {
        // Assume max velocity is positive
        return RadiansPerSecond.of((maxVoltage.in(Volts) - kS - Math.cos(angle.in(Radians)) * kCos - acceleration.in(RadiansPerSecondPerSecond) * kA) / kV);
    }

    /**
     * Calculates the minimum achievable velocity given a maximum voltage supply,
     * a position, and an acceleration.  Useful for ensuring that velocity and
     * acceleration constraints for a trapezoidal profile are simultaneously
     * achievable - enter the acceleration constraint, and this will give you
     * a simultaneously-achievable velocity constraint.
     *
     * @param maxVoltage   The maximum voltage that can be supplied to the arm.
     * @param angle        The angle of the arm.
     * @param acceleration The acceleration of the arm.
     * @return The minimum possible velocity at the given acceleration and angle.
     */
    public Measure<Velocity<Angle>> minAchievableVelocity(Measure<Voltage> maxVoltage, Measure<Angle> angle, Measure<Velocity<Velocity<Angle>>> acceleration) {
        // Assume min velocity is negative, ks flips sign
        return RadiansPerSecond.of((-maxVoltage.in(Volts) + kS - Math.cos(angle.in(Radians)) * kCos - acceleration.in(RadiansPerSecondPerSecond) * kA) / kV);
    }

    /**
     * Calculates the maximum achievable acceleration given a maximum voltage
     * supply, a position, and a velocity. Useful for ensuring that velocity and
     * acceleration constraints for a trapezoidal profile are simultaneously
     * achievable - enter the velocity constraint, and this will give you
     * a simultaneously-achievable acceleration constraint.
     *
     * @param maxVoltage The maximum voltage that can be supplied to the arm.
     * @param angle      The angle of the arm.
     * @param velocity   The velocity of the arm.
     * @return The maximum possible acceleration at the given velocity.
     */
    public Measure<Velocity<Velocity<Angle>>> maxAchievableAcceleration(Measure<Voltage> maxVoltage, Measure<Angle> angle, Measure<Velocity<Angle>> velocity) {
        double v = velocity.in(RadiansPerSecond);
        return RadiansPerSecondPerSecond.of((maxVoltage.in(Volts) - kS * Math.signum(v) - Math.cos(angle.in(Radians)) * kCos - v * kV) / kA);
    }

    /**
     * Calculates the minimum achievable acceleration given a maximum voltage
     * supply, a position, and a velocity. Useful for ensuring that velocity and
     * acceleration constraints for a trapezoidal profile are simultaneously
     * achievable - enter the velocity constraint, and this will give you
     * a simultaneously-achievable acceleration constraint.
     *
     * @param maxVoltage The maximum voltage that can be supplied to the arm.
     * @param angle      The angle of the arm.
     * @param velocity   The velocity of the arm.
     * @return The minimum possible acceleration at the given velocity.
     */
    public Measure<Velocity<Velocity<Angle>>> minAchievableAcceleration(Measure<Voltage> maxVoltage, Measure<Angle> angle, Measure<Velocity<Angle>> velocity) {
        return maxAchievableAcceleration(maxVoltage.negate(), angle, velocity);
    }

    @Override
    public void setCoefficients(double... coeffs) {
        if (coeffs.length != 4) {
            throw new IllegalArgumentException("expected 4 coefficients, got " + coeffs.length);
        }
        kS = coeffs[0];
        kCos = coeffs[1];
        kV = coeffs[2];
        kA = coeffs[3];
    }

    /**
     * Implicit feedforward of assumed radians unit. Assumed acceleration of zero.
     *
     * @param positionRadians          radians of position setpoint
     * @param velocityRadiansPerSecond radians/sec of velocity setpoint
     * @return controller output
     */
    @Override
    public double calculate(double positionRadians, double velocityRadiansPerSecond) {
        return calculate(Radians.of(positionRadians), RadiansPerSecond.of(velocityRadiansPerSecond));
    }

    @Override
    public void reset() {
        // no-op
    }
}