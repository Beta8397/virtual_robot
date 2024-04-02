package org.murraybridgebunyips.bunyipslib.pid;

import com.qualcomm.robotcore.hardware.PIDFCoefficients;

/**
 * This is a PID controller (https://en.wikipedia.org/wiki/PID_controller)
 * for your robot. Internally, it performs all the calculations for you.
 * You need to tune your values to the appropriate amounts in order
 * to properly utilize these calculations.
 * <p>
 * The equation we will use is:
 * u(t) = kP * e(t) + kI * int(0,t)[e(t')dt'] + kD * e'(t) + kF
 * where e(t) = r(t) - y(t) and r(t) is the setpoint and y(t) is the
 * measured value. If we consider e(t) the positional error, then
 * int(0,t)[e(t')dt'] is the total error and e'(t) is the velocity error.
 * <a href="https://github.com/FTCLib/FTCLib/blob/cedc52cee1bb549324c1ca5059c5feec3c054902/core/src/main/java/com/arcrobotics/ftclib/controller/PIDFController.java">Source</a>
 */
public class PIDFController {
    private double kP, kI, kD, kF;
    private double setPoint;
    private double measuredValue;
    private double minIntegral, maxIntegral;

    private double errorVal_p;
    private double errorVal_v;

    private double totalError;
    private double prevErrorVal;

    private double errorTolerance_p = 0.05;
    private double errorTolerance_v = Double.POSITIVE_INFINITY;

    private double lastTimeStamp;
    private double period;

    /**
     * The base constructor for the PIDF controller
     *
     * @param kp The value of kP for the coefficients.
     * @param ki The value of kI for the coefficients.
     * @param kd The value of kD for the coefficients.
     * @param kf The value of kF for the coefficients.
     */
    public PIDFController(double kp, double ki, double kd, double kf) {
        this(kp, ki, kd, kf, 0, 0);
    }

    /**
     * This is the full constructor for the PIDF controller. Our PIDF controller
     * includes a feed-forward value which is useful for fighting friction and gravity.
     * Our errorVal represents the return of e(t) and prevErrorVal is the previous error.
     *
     * @param kp The value of kP for the coefficients.
     * @param ki The value of kI for the coefficients.
     * @param kd The value of kD for the coefficients.
     * @param kf The value of kF for the coefficients.
     * @param sp The setpoint of the pid control loop.
     * @param pv The measured value of he pid control loop. We want sp = pv, or to the degree
     *           such that sp - pv, or e(t) < tolerance.
     */
    public PIDFController(double kp, double ki, double kd, double kf, double sp, double pv) {
        kP = kp;
        kI = ki;
        kD = kd;
        kF = kf;

        setPoint = sp;
        measuredValue = pv;

        minIntegral = -1.0;
        maxIntegral = 1.0;

        lastTimeStamp = 0;
        period = 0;

        errorVal_p = setPoint - measuredValue;
        reset();
    }

    /**
     * Resets the PIDF controller.
     */
    public void reset() {
        totalError = 0;
        prevErrorVal = 0;
        lastTimeStamp = 0;
    }

    /**
     * Sets the error which is considered tolerable for use with {@link #atSetPoint()}.
     *
     * @param positionTolerance Position error which is tolerable.
     * @param velocityTolerance Velocity error which is tolerable.
     */
    public void setTolerance(double positionTolerance, double velocityTolerance) {
        errorTolerance_p = positionTolerance;
        errorTolerance_v = velocityTolerance;
    }

    /**
     * Returns the current setpoint of the PIDFController.
     *
     * @return The current setpoint.
     */
    public double getSetPoint() {
        return setPoint;
    }

    /**
     * Sets the setpoint for the PIDFController
     *
     * @param sp The desired setpoint.
     */
    public void setSetPoint(double sp) {
        setPoint = sp;
        errorVal_p = setPoint - measuredValue;
        errorVal_v = (errorVal_p - prevErrorVal) / period;
    }

    /**
     * Returns true if the error is within the percentage of the total input range, determined by
     * {@link #setTolerance}.
     *
     * @return Whether the error is within the acceptable bounds.
     */
    public boolean atSetPoint() {
        return Math.abs(errorVal_p) < errorTolerance_p
                && Math.abs(errorVal_v) < errorTolerance_v;
    }

    /**
     * @return the PIDF coefficients
     */
    public double[] getCoefficients() {
        return new double[]{kP, kI, kD, kF};
    }

    /**
     * @return the positional error e(t)
     */
    public double getPositionError() {
        return errorVal_p;
    }

    /**
     * @return the tolerances of the controller
     */
    public double[] getTolerance() {
        return new double[]{errorTolerance_p, errorTolerance_v};
    }

    /**
     * Sets the error which is considered tolerable for use with {@link #atSetPoint()}.
     *
     * @param positionTolerance Position error which is tolerable.
     */
    public void setTolerance(double positionTolerance) {
        setTolerance(positionTolerance, Double.POSITIVE_INFINITY);
    }

    /**
     * @return the velocity error e'(t)
     */
    public double getVelocityError() {
        return errorVal_v;
    }

    /**
     * Calculates the next output of the PIDF controller.
     *
     * @return the next output using the current measured value via
     * {@link #calculate(double)}.
     */
    public double calculate() {
        return calculate(measuredValue);
    }

    /**
     * Calculates the next output of the PIDF controller.
     *
     * @param pv The given measured value.
     * @param sp The given setpoint.
     * @return the next output using the given measured value via
     * {@link #calculate(double)}.
     */
    public double calculate(double pv, double sp) {
        // Set the setpoint to the provided value
        setSetPoint(sp);
        return calculate(pv);
    }

    /**
     * Calculates the control value, u(t).
     *
     * @param pv The current measurement of the process variable.
     * @return the value produced by u(t).
     */
    public double calculate(double pv) {
        prevErrorVal = errorVal_p;

        double currentTimeStamp = System.nanoTime() / 1.0E9;
        if (lastTimeStamp == 0) lastTimeStamp = currentTimeStamp;
        period = currentTimeStamp - lastTimeStamp;
        lastTimeStamp = currentTimeStamp;

        if (measuredValue == pv) {
            errorVal_p = setPoint - measuredValue;
        } else {
            errorVal_p = setPoint - pv;
            measuredValue = pv;
        }

        if (Math.abs(period) > 1.0E-6) {
            errorVal_v = (errorVal_p - prevErrorVal) / period;
        } else {
            errorVal_v = 0;
        }

        /*
         * If total error is the integral from 0 to t of e(t')dt', and
         * e(t) = sp - pv, then the total error, E(t), equals sp*t - pv*t.
         */
        totalError += period * (setPoint - measuredValue);
        totalError = totalError < minIntegral ? minIntegral : Math.min(maxIntegral, totalError);

        // Returns u(t)
        return kP * errorVal_p + kI * totalError + kD * errorVal_v + kF * setPoint;
    }

    /**
     * Set the current controller PID coefficients to the given coefficients.
     *
     * @param kp The value of kP for the coefficients.
     * @param ki The value of kI for the coefficients.
     * @param kd The value of kD for the coefficients.
     * @param kf The value of kF for the coefficients.
     */
    public void setPIDF(double kp, double ki, double kd, double kf) {
        kP = kp;
        kI = ki;
        kD = kd;
        kF = kf;
    }

    /**
     * Set the current controller PID coefficients to the given coefficients.
     *
     * @param coefficients the coefficients to set
     */
    public void setPIDF(PIDFCoefficients coefficients) {
        kP = coefficients.p;
        kI = coefficients.i;
        kD = coefficients.d;
        kF = coefficients.f;
    }

    /**
     * Update the supplied PID coefficients with the current controller values.
     *
     * @param coefficients the coefficients to update
     */
    public void updatePIDF(PIDFCoefficients coefficients) {
        coefficients.p = kP;
        coefficients.i = kI;
        coefficients.d = kD;
        coefficients.f = kF;
    }

    /**
     * Set the bounds for the integral term.
     *
     * @param integralMin The minimum value for the integral term.
     * @param integralMax The maximum value for the integral term.
     */
    public void setIntegrationBounds(double integralMin, double integralMax) {
        minIntegral = integralMin;
        maxIntegral = integralMax;
    }

    /**
     * Clear the integral term.
     */
    public void clearTotalError() {
        totalError = 0;
    }

    public double getP() {
        return kP;
    }

    public void setP(double kp) {
        kP = kp;
    }

    public double getI() {
        return kI;
    }

    public void setI(double ki) {
        kI = ki;
    }

    public double getD() {
        return kD;
    }

    public void setD(double kd) {
        kD = kd;
    }

    public double getF() {
        return kF;
    }

    public void setF(double kf) {
        kF = kf;
    }

    public double getPeriod() {
        return period;
    }
}
