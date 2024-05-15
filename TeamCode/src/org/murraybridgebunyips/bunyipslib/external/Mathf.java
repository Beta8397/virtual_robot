package org.murraybridgebunyips.bunyipslib.external;

import static org.murraybridgebunyips.bunyipslib.external.units.Units.Degrees;
import static org.murraybridgebunyips.bunyipslib.external.units.Units.Radians;
import static org.murraybridgebunyips.bunyipslib.external.units.Units.Seconds;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.util.MathUtil;

import org.murraybridgebunyips.bunyipslib.Reference;
import org.murraybridgebunyips.bunyipslib.external.units.Angle;
import org.murraybridgebunyips.bunyipslib.external.units.Measure;
import org.murraybridgebunyips.bunyipslib.external.units.Time;

import java.util.List;
import java.util.stream.Collectors;

/**
 * Extended math utility functions.
 * <p>
 * This class is a combination of WPILib's
 * <a href="https://github.com/wpilibsuite/allwpilib/blob/dc4c63568a2adbc2acfb5d6a420750236074b6aa/wpimath/src/main/java/edu/wpi/first/math/MathUtil.java">MathUtil</a>
 * and Unity's <a href="https://github.com/Unity-Technologies/UnityCsReference/blob/22a9cc4540dc5efa28ad9f02cd12b37b4b1a21c7/Runtime/Export/Math/Mathf.cs">Mathf</a>.
 *
 * @see Math
 */
public final class Mathf {
    private Mathf() {
    }

    /**
     * Solve a quadratic for x, given coefficients a, b, and c.
     *
     * @param a Coefficient of x^2.
     * @param b Coefficient of x.
     * @param c Constant.
     * @return List of real roots.
     */
    public static List<Double> solveQuadratic(double a, double b, double c) {
        return MathUtil.solveQuadratic(a, b, c);
    }

    /**
     * Solve a quadratic for x, given coefficients a, b, and c.
     *
     * @param a Coefficient of x^2.
     * @param b Coefficient of x.
     * @param c Constant.
     * @return List of real roots.
     */
    public static List<Float> solveQuadratic(float a, float b, float c) {
        return MathUtil.solveQuadratic(a, b, c).stream().map(Double::floatValue).collect(Collectors.toList());
    }

    /**
     * Normalizes the given angle to be within the range of [0, 2pi].
     *
     * @param angle The angle to normalize.
     * @return The normalized angle.
     */
    public static Measure<Angle> normaliseAngle(Measure<Angle> angle) {
        return Radians.of(com.acmerobotics.roadrunner.util.Angle.norm(angle.in(Radians)));
    }

    /**
     * Returns value clamped between low and high boundaries.
     *
     * @param value Value to clamp.
     * @param low   The lower boundary to which to clamp value.
     * @param high  The higher boundary to which to clamp value.
     * @return The clamped value.
     */
    public static int clamp(int value, int low, int high) {
        return Math.max(low, Math.min(value, high));
    }

    /**
     * Returns value clamped between low and high boundaries.
     *
     * @param value Value to clamp.
     * @param low   The lower boundary to which to clamp value.
     * @param high  The higher boundary to which to clamp value.
     * @return The clamped value.
     */
    public static double clamp(double value, double low, double high) {
        return Math.max(low, Math.min(value, high));
    }

    /**
     * Return value clamped between low and high boundaries.
     *
     * @param value Value to clamp.
     * @param low   The lower boundary to which to clamp value.
     * @param high  The higher boundary to which to clamp value.
     * @return The clamped value.
     */
    public static float clamp(float value, float low, float high) {
        return Math.max(low, Math.min(value, high));
    }

    /**
     * Scale a number in the range of {@code x1} to {@code x2}, to the range of {@code y1} to {@code y2}.
     *
     * @param n  number to scale
     * @param x1 lower bound range of n
     * @param x2 upper bound range of n
     * @param y1 lower bound of scale
     * @param y2 upper bound of scale
     * @return a double scaled to a value between y1 and y2, inclusive
     */
    public static double scale(double n, double x1, double x2, double y1, double y2) {
        double m = (y1 - y2) / (x1 - x2);
        double c = y1 - x1 * (y1 - y2) / (x1 - x2);
        // y = mx + c
        return m * n + c;
    }

    /**
     * Scale a number in the range of {@code x1} to {@code x2}, to the range of {@code y1} to {@code y2}.
     *
     * @param n  number to scale
     * @param x1 lower bound range of n
     * @param x2 upper bound range of n
     * @param y1 lower bound of scale
     * @param y2 upper bound of scale
     * @return a float scaled to a value between y1 and y2, inclusive
     */
    public static float scale(float n, float x1, float x2, float y1, float y2) {
        return (float) scale((double) n, x1, x2, y1, y2);
    }

    /**
     * Returns 0.0 if the given value is within the specified range around zero. The remaining range
     * between the deadband and the maximum magnitude is scaled from 0.0 to the maximum magnitude.
     *
     * @param value        Value to clip.
     * @param deadband     Range around zero.
     * @param maxMagnitude The maximum magnitude of the input. Can be infinite.
     * @return The value after the deadband is applied.
     */
    public static double applyDeadband(double value, double deadband, double maxMagnitude) {
        if (Math.abs(value) > deadband) {
            if (maxMagnitude / deadband > 1.0e12) {
                // If max magnitude is sufficiently large, the implementation encounters
                // round-off error.  Implementing the limiting behavior directly avoids
                // the problem.
                return value > 0.0 ? value - deadband : value + deadband;
            }
            if (value > 0.0) {
                // Map deadband to 0 and map max to max.
                //
                // y - y₁ = m(x - x₁)
                // y - y₁ = (y₂ - y₁)/(x₂ - x₁) (x - x₁)
                // y = (y₂ - y₁)/(x₂ - x₁) (x - x₁) + y₁
                //
                // (x₁, y₁) = (deadband, 0) and (x₂, y₂) = (max, max).
                // x₁ = deadband
                // y₁ = 0
                // x₂ = max
                // y₂ = max
                //
                // y = (max - 0)/(max - deadband) (x - deadband) + 0
                // y = max/(max - deadband) (x - deadband)
                // y = max (x - deadband)/(max - deadband)
                return maxMagnitude * (value - deadband) / (maxMagnitude - deadband);
            } else {
                // Map -deadband to 0 and map -max to -max.
                //
                // y - y₁ = m(x - x₁)
                // y - y₁ = (y₂ - y₁)/(x₂ - x₁) (x - x₁)
                // y = (y₂ - y₁)/(x₂ - x₁) (x - x₁) + y₁
                //
                // (x₁, y₁) = (-deadband, 0) and (x₂, y₂) = (-max, -max).
                // x₁ = -deadband
                // y₁ = 0
                // x₂ = -max
                // y₂ = -max
                //
                // y = (-max - 0)/(-max + deadband) (x + deadband) + 0
                // y = max/(max - deadband) (x + deadband)
                // y = max (x + deadband)/(max - deadband)
                return maxMagnitude * (value + deadband) / (maxMagnitude - deadband);
            }
        } else {
            return 0.0;
        }
    }

    /**
     * Returns 0.0 if the given value is within the specified range around zero. The remaining range
     * between the deadband and the maximum magnitude is scaled from 0.0 to the maximum magnitude.
     *
     * @param value        Value to clip.
     * @param deadband     Range around zero.
     * @param maxMagnitude The maximum magnitude of the input. Can be infinite.
     * @return The value after the deadband is applied.
     */
    public static float applyDeadband(float value, float deadband, float maxMagnitude) {
        return (float) applyDeadband((double) value, deadband, maxMagnitude);
    }

    /**
     * Returns 0.0 if the given value is within the specified range around zero. The remaining range
     * between the deadband and 1.0 is scaled from 0.0 to 1.0.
     *
     * @param value    Value to clip.
     * @param deadband Range around zero.
     * @return The value after the deadband is applied.
     */
    public static double applyDeadband(double value, double deadband) {
        return applyDeadband(value, deadband, 1);
    }

    /**
     * Returns 0.0 if the given value is within the specified range around zero. The remaining range
     * between the deadband and 1.0 is scaled from 0.0 to 1.0.
     *
     * @param value    Value to clip.
     * @param deadband Range around zero.
     * @return The value after the deadband is applied.
     */
    public static float applyDeadband(float value, float deadband) {
        return (float) applyDeadband((double) value, deadband, 1);
    }

    /**
     * Returns modulus of input.
     *
     * @param input        Input value to wrap.
     * @param minimumInput The minimum value expected from the input.
     * @param maximumInput The maximum value expected from the input.
     * @return The wrapped value.
     */
    public static double inputModulus(double input, double minimumInput, double maximumInput) {
        double v = input;
        double modulus = maximumInput - minimumInput;

        // Wrap input if it's above the maximum input
        int numMax = (int) ((v - minimumInput) / modulus);
        v -= numMax * modulus;

        // Wrap input if it's below the minimum input
        int numMin = (int) ((v - maximumInput) / modulus);
        v -= numMin * modulus;

        return v;
    }

    /**
     * Returns modulus of input.
     *
     * @param input        Input value to wrap.
     * @param minimumInput The minimum value expected from the input.
     * @param maximumInput The maximum value expected from the input.
     * @return The wrapped value.
     */
    public static float inputModulus(float input, float minimumInput, float maximumInput) {
        return (float) inputModulus((double) input, minimumInput, maximumInput);
    }

    /**
     * Wraps an angle to the range -pi to pi radians.
     *
     * @param angle Angle to wrap.
     * @return The wrapped angle.
     */
    public static Measure<Angle> angleModulus(Measure<Angle> angle) {
        return Radians.of(inputModulus(angle.in(Radians), -Math.PI, Math.PI));
    }

    /**
     * Perform linear interpolation between two values.
     *
     * @param startValue The value to start at.
     * @param endValue   The value to end at.
     * @param t          How far between the two values to interpolate. This is clamped to [0, 1].
     * @return The interpolated value.
     */
    public static double interpolate(double startValue, double endValue, double t) {
        return startValue + (endValue - startValue) * clamp(t, 0, 1);
    }

    /**
     * Perform linear interpolation between two values.
     *
     * @param startValue The value to start at.
     * @param endValue   The value to end at.
     * @param t          How far between the two values to interpolate. This is clamped to [0, 1].
     * @return The interpolated value.
     */
    public static float interpolate(float startValue, float endValue, float t) {
        return startValue + (endValue - startValue) * clamp(t, 0, 1);
    }

    /**
     * Perform linear interpolation between two values.
     *
     * @param startValue The value to start at.
     * @param endValue   The value to end at.
     * @param t          How far between the two values to interpolate.
     * @return The interpolated value.
     */
    public static double interpolateUnclamped(double startValue, double endValue, double t) {
        return startValue + (endValue - startValue) * t;
    }

    /**
     * Perform linear interpolation between two values.
     *
     * @param startValue The value to start at.
     * @param endValue   The value to end at.
     * @param t          How far between the two values to interpolate.
     * @return The interpolated value.
     */
    public static float interpolateUnclamped(float startValue, float endValue, float t) {
        return startValue + (endValue - startValue) * t;
    }

    /**
     * Perform linear interpolation like {@link #interpolate(double, double, double)}, but interpolates correctly when
     * they wrap around 1 revolution (360 degrees).
     *
     * @param a The start angle.
     * @param b The end angle.
     * @param t The interpolation parameter.
     * @return The interpolated value.
     */
    public static Measure<Angle> interpolateAngle(Measure<Angle> a, Measure<Angle> b, double t) {
        double delta = repeat(b.in(Degrees) - a.in(Degrees), 360);
        if (delta > 180)
            delta -= 360;
        return Degrees.of(a.in(Degrees) + delta * clamp(t, 0, 1));
    }

    /**
     * Moves a value {@code current} towards {@code target}.
     *
     * @param current  The current value.
     * @param target   The value to move towards.
     * @param maxDelta The maximum change that should be applied to the value.
     * @return The new value.
     */
    public static double moveTowards(double current, double target, double maxDelta) {
        if (Math.abs(target - current) <= maxDelta)
            return target;
        return current + Math.signum(target - current) * maxDelta;
    }

    /**
     * Moves a value {@code current} towards {@code target}.
     *
     * @param current  The current value.
     * @param target   The value to move towards.
     * @param maxDelta The maximum change that should be applied to the value.
     * @return The new value.
     */
    public static float moveTowards(float current, float target, float maxDelta) {
        return (float) moveTowards((double) current, target, maxDelta);
    }

    /**
     * Same as {@link #moveTowards(double, double, double)}, but makes sure the values interpolate correctly when they
     * wrap around 1 revolution (360 degrees).
     *
     * @param current  The current angle.
     * @param target   The angle to move towards.
     * @param maxDelta The maximum change that should be applied to the value.
     * @return The new value.
     */
    public static Measure<Angle> moveTowardsAngle(Measure<Angle> current, Measure<Angle> target, Measure<Angle> maxDelta) {
        Measure<Angle> delta = deltaAngle(current, target);
        if (maxDelta.negate().lt(delta) && delta.lt(maxDelta))
            return target;
        return Degrees.of(moveTowards(current.in(Degrees), current.plus(delta).in(Degrees), maxDelta.in(Degrees)));
    }

    /**
     * Interpolates between min and max with smoothing at the limits.
     *
     * @param from The start value.
     * @param to   The end value.
     * @param t    The interpolation value between the two.
     * @return The smooth step value.
     */
    public static double smoothStep(double from, double to, double t) {
        double newT = clamp(t, 0, 1);
        newT = -2.0F * newT * newT * newT + 3.0F * newT * newT;
        return to * newT + from * (1.0 - newT);
    }

    /**
     * Interpolates between min and max with smoothing at the limits.
     *
     * @param from The start value.
     * @param to   The end value.
     * @param t    The interpolation value between the two.
     * @return The smooth step value.
     */
    public static float smoothStep(float from, float to, float t) {
        return (float) smoothStep((double) from, to, t);
    }

    /**
     * Applies gamma correction to a given value within a specified range.
     *
     * @param value  the input value to be gamma corrected
     * @param absMax the maximum absolute value allowed in the input range
     * @param gamma  the gamma value for correction
     * @return the gamma corrected value
     */
    public static double gamma(double value, double absMax, double gamma) {
        boolean negative = value < 0.0;
        double absVal = Math.abs(value);
        if (absVal > absMax)
            return negative ? -absVal : absVal;
        double result = Math.pow(absVal / absMax, gamma) * absMax;
        return negative ? -result : result;
    }

    /**
     * Applies gamma correction to a given value within a specified range.
     *
     * @param value  the input value to be gamma corrected
     * @param absMax the maximum absolute value allowed in the input range
     * @param gamma  the gamma value for correction
     * @return the gamma corrected value
     */
    public static float gamma(float value, float absMax, float gamma) {
        return (float) gamma((double) value, absMax, gamma);
    }

    /**
     * Gradually changes a value towards a desired goal over time.
     *
     * @param current         The current position.
     * @param target          The position we want to be at.
     * @param currentVelocity The current velocity of the current position moving towards the target.
     *                        This ref is modified by this function and should be passed back into it on subsequent calls.
     * @param smoothTime      Approximately the time it will take to reach the target.
     * @param maxVelocity     The maximum velocity that may be achieved when moving current->target.
     * @param deltaTime       The time since the last call to this method.
     * @return The new position following the smooth damp. The new velocity is stored in the currentVelocity reference,
     * for when this method is called again. Any clamping of this value to minimum limits is left to your discretion.
     */
    public static double smoothDamp(double current, double target, @NonNull Reference<Double> currentVelocity, Measure<Time> smoothTime, double maxVelocity, Measure<Time> deltaTime) {
        double t = smoothTime.in(Seconds);
        double dt = deltaTime.in(Seconds);
        double omega = 2.0 / t;

        // Exponential decay function
        double x = omega * dt;
        double exp = 1.0 / (1.0 + x + 0.48 * x * x + 0.235 * x * x * x);
        double delta = current - target;

        // Clamp maximum speed
        double maxDelta = maxVelocity * t;
        delta = clamp(delta, -maxDelta, maxDelta);

        // Calculate new velocity and output of the current position
        currentVelocity.ifNotPresent(() -> currentVelocity.set(0.0));
        double temp = (currentVelocity.require() + omega * delta) * dt;
        currentVelocity.set((currentVelocity.require() - omega * temp) * exp);
        double output = (current - delta) + (delta + temp) * exp;

        // Prevent overshooting
        if (target - current > 0.0 == output > target) {
            output = target;
            currentVelocity.set((output - target) / dt);
        }

        return output;
    }

    /**
     * Gradually changes a value towards a desired goal over time.
     *
     * @param current         The current position.
     * @param target          The position we want to be at.
     * @param currentVelocity The current velocity of the current position moving towards the target.
     *                        This ref is modified by this function and should be passed back into it on subsequent calls.
     * @param smoothTime      Approximately the time it will take to reach the target.
     * @param maxVelocity     The maximum velocity that may be achieved when moving current->target.
     * @param deltaTime       The time since the last call to this method.
     * @return The new position following the smooth damp. The new velocity is stored in the currentVelocity reference,
     * for when this method is called again. Any clamping of this value to minimum limits is left to your discretion.
     */

    public static float smoothDamp(float current, float target, Reference<Double> currentVelocity, Measure<Time> smoothTime, float maxVelocity, Measure<Time> deltaTime) {
        double res = smoothDamp((double) current, target, currentVelocity, smoothTime, maxVelocity, deltaTime);
        return (float) res;
    }

    /**
     * Gradually changes an angle towards a desired goal over time.
     *
     * @param current         The current angle.
     * @param target          The angle we want to be at.
     * @param currentVelocity The current angular velocity of the current position moving towards the target.
     *                        This ref is modified by this function and should be passed back into it on subsequent calls.
     * @param smoothTime      Approximately the time it will take to reach the target.
     * @param maxVelocity     The maximum velocity that may be achieved when moving current->target.
     * @param deltaTime       The time since the last call to this method.
     * @return The new angle following the smooth damp. The new ang. velocity is stored in the currentVelocity reference,
     * for when this method is called again. Any clamping of this value to minimum limits is left to your discretion.
     */

    public static Measure<Angle> smoothDampAngle(Measure<Angle> current, Measure<Angle> target, Reference<Double> currentVelocity, Measure<Time> smoothTime, double maxVelocity, Measure<Time> deltaTime) {
        double res = smoothDamp(current.in(Degrees), current.plus(deltaAngle(current, target)).in(Degrees), currentVelocity, smoothTime, maxVelocity, deltaTime);
        return Degrees.of(res);
    }

    /**
     * Loops the value t, so that it is never larger than length and never smaller than 0.
     *
     * @param t      The value to loop.
     * @param length The length of the loop.
     * @return The looped value.
     */
    public static double repeat(double t, double length) {
        return clamp(t - Math.floor(t / length) * length, 0.0f, length);
    }

    /**
     * Loops the value t, so that it is never larger than length and never smaller than 0.
     *
     * @param t      The value to loop.
     * @param length The length of the loop.
     * @return The looped value.
     */
    public static float repeat(float t, float length) {
        return (float) repeat((double) t, length);
    }

    /**
     * PingPongs (bounces) the value t, so that it is never larger than length and never smaller than 0.
     *
     * @param t      The value to pingpong.
     * @param length The length of the pingpong.
     * @return The pingponged value.
     */
    public static double pingPong(double t, double length) {
        double repeat = repeat(t, length * 2.0);
        return length - Math.abs(repeat - length);
    }

    /**
     * PingPongs (bounces) the value t, so that it is never larger than length and never smaller than 0.
     *
     * @param t      The value to pingpong.
     * @param length The length of the pingpong.
     * @return The pingponged value.
     */
    public static float pingPong(float t, float length) {
        return (float) pingPong((double) t, length);
    }

    /**
     * Calculates the shortest difference between two given angles.
     *
     * @param current The current angle.
     * @param target  The target angle.
     * @return The shortest difference between the two angles.
     */
    public static Measure<Angle> deltaAngle(Measure<Angle> current, Measure<Angle> target) {
        double delta = repeat(target.minus(current).in(Degrees), 360.0);
        if (delta > 180.0F)
            delta -= 360.0F;
        return Degrees.of(delta);
    }

    /**
     * Find the intersection of two lines.
     *
     * @param p1 The first point of the first line.
     * @param p2 The second point of the first line.
     * @param p3 The first point of the second line.
     * @param p4 The second point of the second line.
     * @return The intersection point.
     * @throws NoInterceptException If no intercept is found.
     */
    public static Vector2d lineIntersection(Vector2d p1, Vector2d p2, Vector2d p3, Vector2d p4) throws NoInterceptException {
        double bx = p2.getX() - p1.getX();
        double by = p2.getY() - p1.getY();
        double dx = p4.getX() - p3.getX();
        double dy = p4.getY() - p3.getY();

        double bDotDPerp = bx * dy - by * dx;
        if (bDotDPerp == 0) {
            throw new NoInterceptException();
        }
        double cx = p3.getX() - p1.getX();
        double cy = p3.getY() - p1.getY();
        double t = (cx * dy - cy * dx) / bDotDPerp;

        return new Vector2d(p1.getX() + t * bx, p1.getY() + t * by);
    }

    /**
     * Find the intersection of two line segments.
     *
     * @param p1 The first point of the first line segment.
     * @param p2 The second point of the first line segment.
     * @param p3 The first point of the second line segment.
     * @param p4 The second point of the second line segment.
     * @return The intersection point.
     * @throws NoInterceptException If no intercept is found.
     */
    public static Vector2d lineSegmentIntersection(Vector2d p1, Vector2d p2, Vector2d p3, Vector2d p4) throws NoInterceptException {
        double bx = p2.getX() - p1.getX();
        double by = p2.getY() - p1.getY();
        double dx = p4.getX() - p3.getX();
        double dy = p4.getY() - p3.getY();

        double bDotDPerp = bx * dy - by * dx;
        if (bDotDPerp == 0) {
            throw new NoInterceptException();
        }
        double cx = p3.getX() - p1.getX();
        double cy = p3.getY() - p1.getY();
        double t = (cx * dy - cy * dx) / bDotDPerp;
        if (t < 0 || t > 1) {
            throw new NoInterceptException();
        }
        double u = (cx * by - cy * bx) / bDotDPerp;
        if (u < 0 || u > 1) {
            throw new NoInterceptException();
        }

        return new Vector2d(p1.getX() + t * bx, p1.getY() + t * by);
    }

    /**
     * Returns the next power of two that is equal to or larger than the specified value.
     *
     * @param value The value.
     * @return The next power of two.
     */
    public static int nextPowerOfTwo(int value) {
        int i = value;
        i -= 1;
        i |= i >> 16;
        i |= i >> 8;
        i |= i >> 4;
        i |= i >> 2;
        i |= i >> 1;
        return i + 1;
    }

    /**
     * Returns the closest power of two that is equal to or larger than the specified value.
     *
     * @param value The value.
     * @return The closest power of two.
     */
    public static int closestPowerOfTwo(int value) {
        int nextPower = nextPowerOfTwo(value);
        int prevPower = nextPower >> 1;
        if (value - prevPower < nextPower - value)
            return prevPower;
        else
            return nextPower;
    }

    /**
     * Returns whether the given value is a power of two.
     *
     * @param value The value.
     * @return Whether the value is a power of two.
     */
    public static boolean isPowerOfTwo(int value) {
        return (value & (value - 1)) == 0;
    }

    /**
     * Return where within interpolation range [0, 1] q is between startValue and endValue.
     *
     * @param startValue Lower part of interpolation range.
     * @param endValue   Upper part of interpolation range.
     * @param q          Query.
     * @return Interpolant in range [0, 1].
     */
    public static double inverseInterpolate(double startValue, double endValue, double q) {
        double totalRange = endValue - startValue;
        if (totalRange <= 0) {
            return 0.0;
        }
        double queryToStart = q - startValue;
        if (queryToStart <= 0) {
            return 0.0;
        }
        return queryToStart / totalRange;
    }

    /**
     * Return where within interpolation range [0, 1] q is between startValue and endValue.
     *
     * @param startValue Lower part of interpolation range.
     * @param endValue   Upper part of interpolation range.
     * @param q          Query.
     * @return Interpolant in range [0, 1].
     */
    public static float inverseInterpolate(float startValue, float endValue, float q) {
        return (float) inverseInterpolate((double) startValue, endValue, q);
    }

    /**
     * Checks if the given value matches an expected value within a certain tolerance.
     *
     * @param expected  The expected value
     * @param actual    The actual value
     * @param tolerance The allowed difference between the actual and the expected value
     * @return Whether or not the actual value is within the allowed tolerance
     */
    public static boolean isNear(double expected, double actual, double tolerance) {
        if (tolerance < 0) {
            throw new IllegalArgumentException("Tolerance must be a non-negative number!");
        }
        return Math.abs(expected - actual) < tolerance;
    }

    /**
     * Checks if the given value matches an expected value within a certain tolerance.
     *
     * @param expected  The expected value
     * @param actual    The actual value
     * @param tolerance The allowed difference between the actual and the expected value
     * @return Whether or not the actual value is within the allowed tolerance
     */
    public static boolean isNear(float expected, float actual, float tolerance) {
        return isNear((double) expected, actual, tolerance);
    }

    /**
     * Checks if the given value matches an expected value within a certain tolerance. Supports
     * continuous input for cases like absolute encoders.
     *
     * <p>Continuous input means that the min and max value are considered to be the same point, and
     * tolerances can be checked across them. A common example would be for absolute encoders: calling
     * isNear(2, 359, 5, 0, 360) returns true because 359 is 1 away from 360 (which is treated as the
     * same as 0) and 2 is 2 away from 0, adding up to an error of 3 degrees, which is within the
     * given tolerance of 5.
     *
     * @param expected  The expected value
     * @param actual    The actual value
     * @param tolerance The allowed difference between the actual and the expected value
     * @param min       Smallest value before wrapping around to the largest value
     * @param max       Largest value before wrapping around to the smallest value
     * @return Whether or not the actual value is within the allowed tolerance
     */
    public static boolean isNear(double expected, double actual, double tolerance, double min, double max) {
        if (tolerance < 0) {
            throw new IllegalArgumentException("Tolerance must be a non-negative number!");
        }
        // Max error is exactly halfway between the min and max
        double errorBound = (max - min) / 2.0;
        double error = inputModulus(expected - actual, -errorBound, errorBound);
        return Math.abs(error) < tolerance;
    }

    /**
     * Checks if the given value matches an expected value within a certain tolerance. Supports
     * continuous input for cases like absolute encoders.
     *
     * <p>Continuous input means that the min and max value are considered to be the same point, and
     * tolerances can be checked across them. A common example would be for absolute encoders: calling
     * isNear(2, 359, 5, 0, 360) returns true because 359 is 1 away from 360 (which is treated as the
     * same as 0) and 2 is 2 away from 0, adding up to an error of 3 degrees, which is within the
     * given tolerance of 5.
     *
     * @param expected  The expected value
     * @param actual    The actual value
     * @param tolerance The allowed difference between the actual and the expected value
     * @param min       Smallest value before wrapping around to the largest value
     * @param max       Largest value before wrapping around to the smallest value
     * @return Whether or not the actual value is within the allowed tolerance
     */
    public static boolean isNear(float expected, float actual, float tolerance, float min, float max) {
        return isNear((double) expected, actual, tolerance, min, max);
    }

    /**
     * Exception thrown if no intercept of two lines is found.
     * You should be looking for this exception if you're using {@link #lineIntersection(Vector2d, Vector2d, Vector2d, Vector2d)},
     * or {@link #lineSegmentIntersection(Vector2d, Vector2d, Vector2d, Vector2d)}.
     */
    public static class NoInterceptException extends RuntimeException {
        /**
         * Create a new NoInterceptException.
         */
        public NoInterceptException() {
            super("No intercept found");
        }
    }
}