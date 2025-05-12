/*
    SPDX-License-Identifier: MIT

    Copyright (c) 2024 SparkFun Electronics
*/
/*
 * Modified by Team Beta 8397 for use in the virtual_robot simulator
 */
package com.qualcomm.hardware.sparkfun;

import com.qualcomm.hardware.CommonOdometry;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * {@link SparkFunOTOS} is the Java driver for the SparkFun Qwiic Optical Tracking Odometry Sensor
 * (OTOS). This is a port of the Arduino library.
 *
 * @see <a href="https://www.sparkfun.com/products/24904">SparkFun OTOS Product Page</a>
 * @see <a href="https://github.com/sparkfun/SparkFun_Qwiic_OTOS_Arduino_Library/">Arduino Library</a>
 */
public class SparkFunOTOS implements HardwareDevice {

    protected CommonOdometry odo;

    // Default I2C addresses of the Qwiic OTOS
    public static final byte DEFAULT_ADDRESS = 0x17;
    // Minimum scalar value for the linear and angular scalars
    public static final double MIN_SCALAR = 0.872;

    // Maximum scalar value for the linear and angular scalars
    // NOTE: The scalars can be set and get, but will have no effect on function in the simulator.
    public static final double MAX_SCALAR = 1.127;

    protected double linearScalar = 1.0;
    protected double angularScalar = 1.0;


    protected DistanceUnit _distanceUnit = DistanceUnit.INCH;
    protected AngleUnit _angularUnit = AngleUnit.DEGREES;
    protected int calibrationCyclesRemaining = 0;

    protected Pose2D position = new Pose2D(0, 0, 0);
    protected Pose2D velocity = new Pose2D(0, 0, 0);
    protected Pose2D acceleration = new Pose2D(0, 0, 0);

    private Pose2D offset;

    // 2D pose structure, including x and y coordinates and heading angle.
    // Although pose is traditionally used for position and orientation, this
    // structure is also used for velocity and accleration by the OTOS driver
    public static class Pose2D {
        public double x;
        public double y;
        public double h;

        public Pose2D() {
            x = 0.0;
            y = 0.0;
            h = 0.0;
        }

        public Pose2D(double x, double y, double h) {
            this.x = x;
            this.y = y;
            this.h = h;
        }

        public void set(Pose2D pose) {
            this.x = pose.x;
            this.y = pose.y;
            this.h = pose.h;
        }
    }

    // Version register structure
    public static class Version {
        public byte minor;
        public byte major;

        public Version() {
            set((byte) 0);
        }

        public Version(byte value) {
            set(value);
        }

        public void set(byte value) {
            minor = (byte) (value & 0x0F);
            major = (byte) ((value >> 4) & 0x0F);
        }

        public byte get() {
            return (byte) ((major << 4) | minor);
        }
    }

    // Signal process config register structure
    public static class SignalProcessConfig {
        public boolean enLut;
        public boolean enAcc;
        public boolean enRot;
        public boolean enVar;

        public SignalProcessConfig() {
            set((byte) 0);
        }

        public SignalProcessConfig(byte value) {
            set(value);
        }

        public void set(byte value) {
            enLut = (value & 0x01) != 0;
            enAcc = (value & 0x02) != 0;
            enRot = (value & 0x04) != 0;
            enVar = (value & 0x08) != 0;
        }

        public byte get() {
            return (byte) ((enLut ? 0x01 : 0) | (enAcc ? 0x02 : 0) | (enRot ? 0x04 : 0) | (enVar ? 0x08 : 0));
        }
    }

    // Self-test config register structure
    public static class SelfTestConfig {
        public boolean start;
        public boolean inProgress;
        public boolean pass;
        public boolean fail;

        public SelfTestConfig() {
            set((byte) 0);
        }

        public SelfTestConfig(byte value) {
            set(value);
        }

        public void set(byte value) {
            start = (value & 0x01) != 0;
            inProgress = (value & 0x02) != 0;
            pass = (value & 0x04) != 0;
            fail = (value & 0x08) != 0;
        }

        public byte get() {
            return (byte) ((start ? 0x01 : 0) | (inProgress ? 0x02 : 0) | (pass ? 0x04 : 0) | (fail ? 0x08 : 0));
        }
    }

    // Status register structure
    public static class Status {
        public boolean warnTiltAngle;
        public boolean warnOpticalTracking;
        public boolean errorPaa;
        public boolean errorLsm;

        public Status() {
            set((byte) 0);
        }

        public Status(byte value) {
            set(value);
        }

        public void set(byte value) {
            warnTiltAngle = (value & 0x01) != 0;
            warnOpticalTracking = (value & 0x02) != 0;
            errorPaa = (value & 0x40) != 0;
            errorLsm = (value & 0x80) != 0;
        }

        public byte get() {
            return (byte) ((warnTiltAngle ? 0x01 : 0) | (warnOpticalTracking ? 0x02 : 0) | (errorPaa ? 0x40 : 0) | (errorLsm ? 0x80 : 0));
        }
    }

    public SparkFunOTOS(CommonOdometry odo){
        this.odo = odo;
    }


    @Override
    public Manufacturer getManufacturer()
    {
        return Manufacturer.SparkFun;
    }

    @Override
    public String getDeviceName()
    {
        return "SparkFun Qwiic Optical Tracking Odometry Sensor";
    }

    /**
     * Begins the Qwiic OTOS and verifies it is connected
     * @return True if successful, false otherwise
     */
    public boolean begin() {
        // Just check if the device is connected, no other setup is needed
        return isConnected();
    }

    /**
     * Checks if the OTOS is connected to the I2C bus
     * @return True if the OTOS is connected, false otherwise
     */
    public boolean isConnected() {
        return true;
    }

    /**
     * Gets the hardware and firmware version of the OTOS
     * @param hwVersion Hardware version number
     * @param fwVersion Firmware version number
     */
    public void getVersionInfo(Version hwVersion, Version fwVersion) {
        // Read hardware and firmware version registers
        byte[] rawData = new byte[]{0,0};

        // Store the version info
        hwVersion.set(rawData[0]);
        fwVersion.set(rawData[1]);
    }

    /**
     * Performs a self-test on the OTOS
     * @return True if the self-test passed, false otherwise
     */
    public boolean selfTest() {return true;}

    /**
     * Calibrates the IMU on the OTOS, which removes the accelerometer and
     * gyroscope offsets. This will do the full 255 samples and wait until
     * the calibration is done, which takes about 612ms as of firmware v1.0
     * @return True if the calibration was successful, false otherwise
     */
    public boolean calibrateImu() {
        return calibrateImu(255, true);
    }

    /**
     * Calibrates the IMU on the OTOS, which removes the accelerometer and
     * gyroscope offsets
     * @param numSamples Number of samples to take for calibration. Each sample
     * takes about 2.4ms, so fewer samples can be taken for faster calibration
     * @param waitUntilDone Whether to wait until the calibration is complete.
     * Set false to calibrate asynchronously, see getImuCalibrationProgress()
     * @return True if the calibration was successful, false otherwise
     */
    public boolean calibrateImu(int numSamples, boolean waitUntilDone) {
        // Check if the number of samples is out of bounds
        if (numSamples < 1 || numSamples > 255)
            return false;

        try {
            calibrationCyclesRemaining = numSamples;
            long cycleStartNanos = System.nanoTime();
            while (calibrationCyclesRemaining > 0) {
                while (true) {
                    long currentNanos = System.nanoTime();
                    if (currentNanos - cycleStartNanos > 2400) {
                        calibrationCyclesRemaining--;
                        cycleStartNanos = currentNanos;
                        break;
                    }
                    Thread.currentThread().sleep(0);
                }
            }
        } catch(InterruptedException ex){
            return false;
        } finally{
            calibrationCyclesRemaining = 0;
        }

        return true;
    }

    /**
     * Gets the progress of the IMU calibration. Used for asynchronous
     * calibration with calibrateImu()
     * @return Number of samples remaining for calibration
     */
    public int getImuCalibrationProgress() {
        // Read the IMU calibration register
        return calibrationCyclesRemaining;
    }

    /**
     * Gets the linear unit used by all methods using a pose
     * @return Linear unit
     */
    public DistanceUnit getLinearUnit() {
        return _distanceUnit;
    }

    /**
     * Sets the linear unit used by all methods using a pose
     * @param unit Linear unit
     */
    public void setLinearUnit(DistanceUnit unit) {
        // Check if this unit is already set
        if (unit == _distanceUnit)
            return;

        // Store new unit
        _distanceUnit = unit;
    }

    /**
     * Gets the angular unit used by all methods using a pose
     * @return Angular unit
     */
    public AngleUnit getAngularUnit() {
        return _angularUnit;
    }

    /**
     * Sets the angular unit used by all methods using a pose
     * @param unit Angular unit
     */
    public void setAngularUnit(AngleUnit unit) {
        // Check if this unit is already set
        if (unit == _angularUnit)
            return;

        // Store new unit
        _angularUnit = unit;
    }

    /**
     * Gets the linear scalar used by the OTOS
     * @return Linear scalar
     */
    public double getLinearScalar() {
        return linearScalar;
    }

    /**
     * Sets the linear scalar used by the OTOS. Can be used to
     * compensate for scaling issues with the sensor measurements
     * @param scalar Linear scalar, must be between 0.872 and 1.127
     * @return True if the scalar was set successfully, false otherwise
     */
    public boolean setLinearScalar(double scalar) {
        // Check if the scalar is out of bounds
        if (scalar < MIN_SCALAR || scalar > MAX_SCALAR)
            return false;

        linearScalar = scalar;

        // Done!
        return true;
    }

    /**
     * Gets the angular scalar used by the OTOS
     * @return Angular scalar
     */
    public double getAngularScalar() {
        return angularScalar;
    }

    /**
     * Sets the angular scalar used by the OTOS. Can be used to
     * compensate for scaling issues with the sensor measurements
     * @param scalar Angular scalar, must be between 0.872 and 1.127
     * @return True if the scalar was set successfully, false otherwise
     */
    public boolean setAngularScalar(double scalar) {
        // Check if the scalar is out of bounds
        if (scalar < MIN_SCALAR || scalar > MAX_SCALAR)
            return false;

        angularScalar = scalar;

        // Done!
        return true;
    }

    /**
     * Resets the tracking algorithm, which resets the position to the
     * origin, but can also be used to recover from some rare tracking errors
     */
    public void resetTracking() {
        setPosition(new Pose2D(0, 0, 0));
    }

    /**
     * Gets the signal processing configuration from the OTOS
     * @return Signal processing configuration
     */
    public SignalProcessConfig getSignalProcessConfig() {
        // Read the signal process register
        return new SignalProcessConfig();
    }

    /**
     * Sets the signal processing configuration on the OTOS. This is
     * primarily useful for creating and testing a new lookup table calibration
     * @param config Signal processing configuration
     */
    public void setSignalProcessConfig(SignalProcessConfig config) {
    }

    /**
     * Gets the status register from the OTOS, which includes warnings
     * and errors reported by the sensor
     * @return Status register value
     */
    public Status getStatus() {
        return new Status();
    }

    /**
     * Sets the offset of the OTOS. This is useful if your sensor is
     * mounted off-center from a robot. Rather than returning the position of
     * the sensor, the OTOS will return the position of the robot
     * @param pose Offset of the sensor relative to the center of the robot
     */
    public void setOffset(Pose2D pose) {
        this.offset = pose;
    }

    /**
     * Gets the position measured by the OTOS
     * @return Position measured by the OTOS
     */
    public synchronized Pose2D getPosition() {
        return position;
    }

    /**
     * Sets the position measured by the OTOS. This is useful if your
     * robot does not start at the origin, or you have another source of
     * location information (eg. vision odometry); the OTOS will continue
     * tracking from this position
     * @param pose New position for the OTOS to track from
     */
    public synchronized void setPosition(Pose2D pose) {
        org.firstinspires.ftc.robotcore.external.navigation.Pose2D odoPose2D =
                new org.firstinspires.ftc.robotcore.external.navigation.Pose2D(
                        DistanceUnit.METER, DistanceUnit.METER.fromUnit(_distanceUnit, pose.x),
                        DistanceUnit.METER.fromUnit(_distanceUnit, pose.y), AngleUnit.RADIANS,
                        AngleUnit.RADIANS.fromUnit(_angularUnit, pose.h)
                );
        odo.setPosition(odoPose2D);
        position = pose;
    }

    /**
     * Gets the velocity measured by the OTOS
     * @return Velocity measured by the OTOS
     */
    public synchronized Pose2D getVelocity() {
        return velocity;
    }

    /**
     * Gets the acceleration measured by the OTOS
     * @return Acceleration measured by the OTOS
     */
    public synchronized Pose2D getAcceleration() {
        return acceleration;
    }

    /**
     * Gets the standard deviation of the measured position
     * These values are just the square root of the diagonal elements
     * of the covariance matrices of the Kalman filters used in the firmware, so
     * they are just statistical quantities and do not represent actual error!
     * @return Standard deviation of the position measured by the OTOS
     */
    public Pose2D getPositionStdDev() {
        return new Pose2D(0,0,0);
    }

    /**
     * Gets the standard deviation of the measured velocity
     * These values are just the square root of the diagonal elements
     * of the covariance matrices of the Kalman filters used in the firmware, so
     * they are just statistical quantities and do not represent actual error!
     * @return Standard deviation of the velocity measured by the OTOS
     */
    public Pose2D getVelocityStdDev() {
        return new Pose2D(0,0,0);
    }

    /**
     * Gets the standard deviation of the measured acceleration
     * These values are just the square root of the diagonal elements
     * of the covariance matrices of the Kalman filters used in the firmware, so
     * they are just statistical quantities and do not represent actual error!
     * @return Standard deviation of the acceleration measured by the OTOS
     */
    public Pose2D getAccelerationStdDev() {
        return new Pose2D(0,0,0);
    }

    /**
     * Gets the position, velocity, and acceleration measured by the
     * OTOS in a single burst read
     * @param pos Position measured by the OTOS
     * @param vel Velocity measured by the OTOS
     * @param acc Acceleration measured by the OTOS
     */
    public synchronized void getPosVelAcc(Pose2D pos, Pose2D vel, Pose2D acc) {
        pos.set(getPosition());
        vel.set(getVelocity());
        acc.set(getAcceleration());
    }

    /**
     * Gets the standard deviation of the measured position, velocity,
     * and acceleration in a single burst read
     * @param pos Standard deviation of the position measured by the OTOS
     * @param vel Standard deviation of the velocity measured by the OTOS
     * @param acc Standard deviation of the acceleration measured by the OTOS
     */
    public void getPosVelAccStdDev(Pose2D pos, Pose2D vel, Pose2D acc) {
        pos.set(new Pose2D(0,0,0));
        vel.set(new Pose2D(0,0,0));
        acc.set(new Pose2D(0,0,0));
    }

    /**
     * Gets the position, velocity, acceleration, and standard deviation
     * of each in a single burst read
     * @param pos Position measured by the OTOS
     * @param vel Velocity measured by the OTOS
     * @param acc Acceleration measured by the OTOS
     * @param posStdDev Standard deviation of the position measured by the OTOS
     * @param velStdDev Standard deviation of the velocity measured by the OTOS
     * @param accStdDev Standard deviation of the acceleration measured by the OTOS
     */
    public synchronized void getPosVelAccAndStdDev(Pose2D pos, Pose2D vel, Pose2D acc,
                                      Pose2D posStdDev, Pose2D velStdDev, Pose2D accStdDev) {
        getPosVelAcc(pos, vel, acc);
        posStdDev.set(new Pose2D(0,0,0));
        velStdDev.set(new Pose2D(0,0,0));
        accStdDev.set(new Pose2D(0,0,0));
    }
}
