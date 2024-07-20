/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.murraybridgebunyips.bunyipslib.external;

import org.murraybridgebunyips.bunyipslib.external.units.Measure;
import org.murraybridgebunyips.bunyipslib.external.units.Unit;
import org.murraybridgebunyips.bunyipslib.external.units.Velocity;

import java.util.Objects;


/**
 * A trapezoid-shaped velocity profile.
 *
 * <p>While this class can be used for a profiled movement from start to finish,
 * the intended usage is to filter a reference's dynamics based on trapezoidal
 * velocity constraints. To compute the reference obeying this constraint, do
 * the following.
 *
 * <p>Initialization:
 * <pre>{@code
 * TrapezoidProfile.Constraints constraints =
 *   new TrapezoidProfile.Constraints(kMaxV, kMaxA);
 * TrapezoidProfile.State previousProfiledReference =
 *   new TrapezoidProfile.State(initialReference, 0.0);
 * }</pre>
 *
 * <p>Run on update:
 * <pre>{@code
 * TrapezoidProfile profile =
 *   new TrapezoidProfile(constraints, unprofiledReference, previousProfiledReference);
 * previousProfiledReference = profile.calculate(timeSincePreviousUpdate);
 * }</pre>
 *
 * <p>where `unprofiledReference` is free to change between calls. Note that when
 * the unprofiled reference is within the constraints, `calculate()` returns the
 * unprofiled reference unchanged.
 *
 * <p>Otherwise, a timer can be started to provide monotonic values for
 * `calculate()` and to determine when the profile has completed via
 * `isFinished()`.
 * <a href="https://github.com/FTCLib/FTCLib/blob/1c8995d09413b406e0f4aff238ea4edc2bb860c4/core/src/main/java/com/arcrobotics/ftclib/trajectory/TrapezoidProfile.java">Source</a>
 */
public class TrapezoidProfile {
    // The direction of the profile, either 1 for forwards or -1 for inverted
    private final int direction;

    private final Constraints constraints;
    private final State initial;
    private final State goal;

    private final double endAccel;
    private final double endFullSpeed;
    private final double endDeccel;

    /**
     * Construct a TrapezoidProfile.
     *
     * @param constraints The constraints on the profile, like maximum velocity.
     * @param goal        The desired state when the profile is complete.
     * @param initial     The initial state (usually the current state).
     */
    public TrapezoidProfile(Constraints constraints, State goal, State initial) {
        direction = shouldFlipAcceleration(initial, goal) ? -1 : 1;
        this.constraints = constraints;
        this.initial = direct(initial);
        this.goal = direct(goal);

        if (this.initial.velocity > this.constraints.maxVelocity) {
            this.initial.velocity = this.constraints.maxVelocity;
        }

        // Deal with a possibly truncated motion profile (with nonzero initial or
        // final velocity) by calculating the parameters as if the profile began and
        // ended at zero velocity
        double cutoffBegin = this.initial.velocity / this.constraints.maxAcceleration;
        double cutoffDistBegin = cutoffBegin * cutoffBegin * this.constraints.maxAcceleration / 2.0;

        double cutoffEnd = this.goal.velocity / this.constraints.maxAcceleration;
        double cutoffDistEnd = cutoffEnd * cutoffEnd * this.constraints.maxAcceleration / 2.0;

        // Now we can calculate the parameters as if it was a full trapezoid instead
        // of a truncated one

        double fullTrapezoidDist = cutoffDistBegin + (this.goal.position - this.initial.position)
                + cutoffDistEnd;
        double accelerationTime = this.constraints.maxVelocity / this.constraints.maxAcceleration;

        double fullSpeedDist = fullTrapezoidDist - accelerationTime * accelerationTime
                * this.constraints.maxAcceleration;

        // Handle the case where the profile never reaches full speed
        if (fullSpeedDist < 0) {
            accelerationTime = Math.sqrt(fullTrapezoidDist / this.constraints.maxAcceleration);
            fullSpeedDist = 0;
        }

        endAccel = accelerationTime - cutoffBegin;
        endFullSpeed = endAccel + fullSpeedDist / this.constraints.maxVelocity;
        endDeccel = endFullSpeed + accelerationTime - cutoffEnd;
    }

    /**
     * Construct a TrapezoidProfile.
     *
     * @param constraints The constraints on the profile, like maximum velocity.
     * @param goal        The desired state when the profile is complete.
     */
    public TrapezoidProfile(Constraints constraints, State goal) {
        this(constraints, goal, new State(0, 0));
    }

    /**
     * Returns true if the profile inverted.
     *
     * <p>The profile is inverted if goal position is less than the initial position.
     *
     * @param initial The initial state (usually the current state).
     * @param goal    The desired state when the profile is complete.
     */
    private static boolean shouldFlipAcceleration(State initial, State goal) {
        return initial.position > goal.position;
    }

    /**
     * Calculate the correct position and velocity for the profile at a time t
     * where the beginning of the profile was at time t = 0.
     *
     * @param t The time since the beginning of the profile.
     * @return The state (position and velocity) at time t.
     */
    public State calculate(double t) {
        State result = new State(initial.position, initial.velocity);

        if (t < endAccel) {
            result.velocity += t * constraints.maxAcceleration;
            result.position += (initial.velocity + t * constraints.maxAcceleration / 2.0) * t;
        } else if (t < endFullSpeed) {
            result.velocity = constraints.maxVelocity;
            result.position += (initial.velocity + endAccel * constraints.maxAcceleration
                    / 2.0) * endAccel + constraints.maxVelocity * (t - endAccel);
        } else if (t <= endDeccel) {
            result.velocity = goal.velocity + (endDeccel - t) * constraints.maxAcceleration;
            double timeLeft = endDeccel - t;
            result.position = goal.position - (goal.velocity + timeLeft
                    * constraints.maxAcceleration / 2.0) * timeLeft;
        } else {
            result = goal;
        }

        return direct(result);
    }

    /**
     * Returns the time left until a target distance in the profile is reached.
     *
     * @param target The target distance.
     * @return The time left until the target distance is reached.
     */
    public double timeLeftUntil(double target) {
        double position = initial.position * direction;
        double velocity = initial.velocity * direction;

        double endAccel = this.endAccel * direction;
        double endFullSpeed = this.endFullSpeed * direction - endAccel;

        if (target < position) {
            endAccel = -endAccel;
            endFullSpeed = -endFullSpeed;
            velocity = -velocity;
        }

        endAccel = Math.max(endAccel, 0);
        endFullSpeed = Math.max(endFullSpeed, 0);
        double endDeccel = this.endDeccel - endAccel - endFullSpeed;

        double acceleration = constraints.maxAcceleration;
        double decceleration = -constraints.maxAcceleration;

        double distToTarget = Math.abs(target - position);
        if (distToTarget < 1.0e-6) {
            return 0;
        }

        double accelDist = velocity * endAccel + 0.5 * acceleration * endAccel * endAccel;

        double deccelVelocity;
        if (endAccel > 0) {
            deccelVelocity = Math.sqrt(Math.abs(velocity * velocity + 2 * acceleration * accelDist));
        } else {
            deccelVelocity = velocity;
        }

        double deccelDist;
        double fullSpeedDist = constraints.maxVelocity * endFullSpeed;

        if (accelDist > distToTarget) {
            accelDist = distToTarget;
            fullSpeedDist = 0;
            deccelDist = 0;
        } else if (accelDist + fullSpeedDist > distToTarget) {
            fullSpeedDist = distToTarget - accelDist;
            deccelDist = 0;
        } else {
            deccelDist = distToTarget - fullSpeedDist - accelDist;
        }

        double accelTime = (-velocity + Math.sqrt(Math.abs(velocity * velocity + 2 * acceleration
                * accelDist))) / acceleration;

        double deccelTime = (-deccelVelocity + Math.sqrt(Math.abs(deccelVelocity * deccelVelocity
                + 2 * decceleration * deccelDist))) / decceleration;

        double fullSpeedTime = fullSpeedDist / constraints.maxVelocity;

        return accelTime + fullSpeedTime + deccelTime;
    }

    /**
     * Returns the total time the profile takes to reach the goal.
     *
     * @return The total time the profile takes to reach the goal.
     */
    public double totalTime() {
        return endDeccel;
    }

    /**
     * Returns true if the profile has reached the goal.
     *
     * <p>The profile has reached the goal if the time since the profile started
     * has exceeded the profile's total time.
     *
     * @param t The time since the beginning of the profile.
     * @return Whether the profile has reached the goal.
     */
    public boolean isFinished(double t) {
        return t >= totalTime();
    }

    // Flip the sign of the velocity and position if the profile is inverted
    private State direct(State in) {
        State result = new State(in.position, in.velocity);
        result.position = result.position * direction;
        result.velocity = result.velocity * direction;
        return result;
    }

    /**
     * Profile constraints.
     */
    public static class Constraints {
        /**
         * Maximum velocity.
         */
        public final double maxVelocity;

        /**
         * Maximum acceleration.
         */
        public final double maxAcceleration;

        /**
         * Constructs constraints for a TrapezoidProfile.
         *
         * @param maxVelocity     maximum velocity
         * @param maxAcceleration maximum acceleration
         */
        public Constraints(double maxVelocity, double maxAcceleration) {
            this.maxVelocity = maxVelocity;
            this.maxAcceleration = maxAcceleration;
        }

        /**
         * Constructs constraints for a TrapezoidProfile.
         *
         * @param <U>             Unit type.
         * @param maxVelocity     maximum velocity
         * @param maxAcceleration maximum acceleration
         */
        public <U extends Unit<U>> Constraints(
                Measure<Velocity<U>> maxVelocity, Measure<Velocity<Velocity<U>>> maxAcceleration) {
            this(maxVelocity.baseUnitMagnitude(), maxAcceleration.baseUnitMagnitude());
        }
    }

    /**
     * Profile state.
     */
    public static class State {
        /**
         * The position at this state.
         */
        public double position;

        /**
         * The velocity at this state.
         */
        public double velocity;

        /**
         * Default constructor.
         */
        public State() {
        }

        /**
         * Constructs constraints for a Trapezoid Profile.
         *
         * @param position The position at this state.
         * @param velocity The velocity at this state.
         */
        public State(double position, double velocity) {
            this.position = position;
            this.velocity = velocity;
        }

        /**
         * Constructs constraints for a Trapezoid Profile.
         *
         * @param <U>      Unit type.
         * @param position The position at this state.
         * @param velocity The velocity at this state.
         */
        public <U extends Unit<U>> State(Measure<U> position, Measure<Velocity<U>> velocity) {
            this(position.baseUnitMagnitude(), velocity.baseUnitMagnitude());
        }

        @Override
        public boolean equals(Object other) {
            if (other instanceof State) {
                State rhs = (State) other;
                return position == rhs.position
                        && velocity == rhs.velocity;
            }
            return false;
        }

        @Override
        public int hashCode() {
            return Objects.hash(position, velocity);
        }
    }
}