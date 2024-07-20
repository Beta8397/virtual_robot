/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.murraybridgebunyips.bunyipslib.external.pid;

import org.murraybridgebunyips.bunyipslib.external.TrapezoidProfile;

/**
 * Implements a PID control loop whose setpoint is constrained by a trapezoid profile.
 * <a href="https://github.com/FTCLib/FTCLib/blob/1c8995d09413b406e0f4aff238ea4edc2bb860c4/core/src/main/java/com/arcrobotics/ftclib/controller/wpilibcontroller/ProfiledPIDController.java">Source</a>
 */
public class ProfiledPIDController {
    private final PIDController controller;
    private TrapezoidProfile.State goal = new TrapezoidProfile.State();
    private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();
    private TrapezoidProfile.Constraints constraints;

    /**
     * Allocates a ProfiledPIDController with the given constants for Kp, Ki, and
     * Kd.
     *
     * @param Kp          The proportional coefficient.
     * @param Ki          The integral coefficient.
     * @param Kd          The derivative coefficient.
     * @param constraints Velocity and acceleration constraints for goal.
     */
    public ProfiledPIDController(double Kp, double Ki, double Kd,
                                 TrapezoidProfile.Constraints constraints) {
        controller = new PIDController(Kp, Ki, Kd);
        this.constraints = constraints;
    }

    /**
     * Sets the PID Controller gain parameters.
     *
     * <p>Sets the proportional, integral, and differential coefficients.
     *
     * @param Kp Proportional coefficient
     * @param Ki Integral coefficient
     * @param Kd Differential coefficient
     */
    public void setPID(double Kp, double Ki, double Kd) {
        controller.setPID(Kp, Ki, Kd);
    }

    /**
     * Gets the proportional coefficient.
     *
     * @return proportional coefficient
     */
    public double getP() {
        return controller.getP();
    }

    /**
     * Sets the proportional coefficient of the PID controller gain.
     *
     * @param Kp proportional coefficient
     */
    public void setP(double Kp) {
        controller.setP(Kp);
    }

    /**
     * Gets the integral coefficient.
     *
     * @return integral coefficient
     */
    public double getI() {
        return controller.getI();
    }

    /**
     * Sets the integral coefficient of the PID controller gain.
     *
     * @param Ki integral coefficient
     */
    public void setI(double Ki) {
        controller.setI(Ki);
    }

    /**
     * Gets the differential coefficient.
     *
     * @return differential coefficient
     */
    public double getD() {
        return controller.getD();
    }

    /**
     * Sets the differential coefficient of the PID controller gain.
     *
     * @param Kd differential coefficient
     */
    public void setD(double Kd) {
        controller.setD(Kd);
    }

    /**
     * Gets the period of this controller.
     *
     * @return The period of the controller.
     */
    public double getPeriod() {
        return controller.getPeriod();
    }

    /**
     * Gets the goal for the ProfiledPIDController.
     */
    public TrapezoidProfile.State getGoal() {
        return goal;
    }

    /**
     * Sets the goal for the ProfiledPIDController.
     *
     * @param goal The desired goal state.
     */
    public void setGoal(TrapezoidProfile.State goal) {
        this.goal = goal;
    }

    /**
     * Sets the goal for the ProfiledPIDController.
     *
     * @param goal The desired goal position.
     */
    public void setGoal(double goal) {
        this.goal = new TrapezoidProfile.State(goal, 0);
    }

    /**
     * Returns true if the error is within the tolerance of the error.
     *
     * <p>This will return false until at least one input value has been computed.
     *
     * @return Whether the error is within the tolerance.
     */
    public boolean atGoal() {
        return atSetpoint() && goal.equals(setpoint);
    }

    /**
     * Set velocity and acceleration constraints for goal.
     *
     * @param constraints Velocity and acceleration constraints for goal.
     */
    public void setConstraints(TrapezoidProfile.Constraints constraints) {
        this.constraints = constraints;
    }

    /**
     * Returns the current setpoint of the ProfiledPIDController.
     *
     * @return The current setpoint.
     */
    public TrapezoidProfile.State getSetpoint() {
        return setpoint;
    }

    /**
     * Returns true if the error is within the tolerance of the error.
     *
     * <p>This will return false until at least one input value has been computed.
     *
     * @return Whether the error is within the tolerance.
     */
    public boolean atSetpoint() {
        return controller.atSetPoint();
    }

    /**
     * Sets the error which is considered tolerable for use with atSetpoint().
     *
     * @param positionTolerance Position error which is tolerable.
     */
    public void setTolerance(double positionTolerance) {
        setTolerance(positionTolerance, Double.POSITIVE_INFINITY);
    }

    /**
     * Sets the error which is considered tolerable for use with atSetpoint().
     *
     * @param positionTolerance Position error which is tolerable.
     * @param velocityTolerance Velocity error which is tolerable.
     */
    public void setTolerance(double positionTolerance, double velocityTolerance) {
        controller.setTolerance(positionTolerance, velocityTolerance);
    }

    /**
     * Returns the difference between the setpoint and the measurement.
     *
     * @return The error.
     */
    public double getPositionError() {
        return controller.getPositionError();
    }

    /**
     * Returns the change in error per second.
     */
    public double getVelocityError() {
        return controller.getVelocityError();
    }

    /**
     * Returns the next output of the PID controller.
     *
     * @param measurement The current measurement of the process variable.
     * @return The next output of the PID controller.
     */
    public double calculate(double measurement) {
        TrapezoidProfile profile = new TrapezoidProfile(constraints, goal, setpoint);
        setpoint = profile.calculate(getPeriod());
        return controller.calculate(measurement, setpoint.position);
    }

    /**
     * Returns the next output of the PID controller.
     *
     * @param measurement The current measurement of the process variable.
     * @param newGoal     The new goal of the controller.
     * @return The next output of the PID controller.
     */
    public double calculate(double measurement, TrapezoidProfile.State newGoal) {
        goal = newGoal;
        return calculate(measurement);
    }

    /**
     * Returns the next output of the PIDController.
     *
     * @param measurement The current measurement of the process variable.
     * @param newGoal     The new goal of the controller.
     * @return The next output of the PID controller.
     */
    public double calculate(double measurement, double newGoal) {
        setGoal(newGoal);
        return calculate(measurement);
    }

    /**
     * Returns the next output of the PID controller.
     *
     * @param measurement    The current measurement of the process variable.
     * @param newGoal        The new goal of the controller.
     * @param newConstraints Velocity and acceleration constraints for goal.
     * @return The next output of the PID controller.
     */
    public double calculate(double measurement, TrapezoidProfile.State newGoal,
                            TrapezoidProfile.Constraints newConstraints) {
        constraints = newConstraints;
        return calculate(measurement, newGoal);
    }

    /**
     * Reset the previous error, the integral term, and disable the controller.
     */
    public void reset() {
        controller.reset();
    }

    /**
     * Reset the previous error and the integral term.
     *
     * @param measurement The current measured State of the system.
     */
    public void reset(TrapezoidProfile.State measurement) {
        controller.reset();
        setpoint = measurement;
    }

    /**
     * Reset the previous error and the integral term.
     *
     * @param measuredPosition The current measured position of the system.
     * @param measuredVelocity The current measured velocity of the system.
     */
    public void reset(double measuredPosition, double measuredVelocity) {
        reset(new TrapezoidProfile.State(measuredPosition, measuredVelocity));
    }

    /**
     * Reset the previous error and the integral term.
     *
     * @param measuredPosition The current measured position of the system. The velocity is
     *                         assumed to be zero.
     */
    public void reset(double measuredPosition) {
        reset(measuredPosition, 0.0);
    }

}