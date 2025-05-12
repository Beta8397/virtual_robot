/*   MIT License
 *   Copyright (c) [2025] [Base 10 Assets, LLC]
 *
 *   Permission is hereby granted, free of charge, to any person obtaining a copy
 *   of this software and associated documentation files (the "Software"), to deal
 *   in the Software without restriction, including without limitation the rights
 *   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *   copies of the Software, and to permit persons to whom the Software is
 *   furnished to do so, subject to the following conditions:

 *   The above copyright notice and this permission notice shall be included in all
 *   copies or substantial portions of the Software.

 *   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *   SOFTWARE.
 */

//Modified by Team Beta 8397 for use in the virtual_robot simulator

package com.qualcomm.hardware.gobilda;

import com.qualcomm.hardware.CommonOdometry;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

@DeviceProperties(
        name = "goBILDA® Pinpoint Odometry Computer",
        xmlTag = "goBILDAPinpoint",
        description ="goBILDA® Pinpoint Odometry Computer (IMU Sensor Fusion for 2 Wheel Odometry)"
        )

public class GoBildaPinpointDriver implements HardwareDevice {

    private final int deviceStatus   = 1;
    private final int loopTime       = 1000;
    protected float xEncoderValue  = 0;
    protected float yEncoderValue  = 0;
    private float xPosition    = 0;
    private float yPosition    = 0;
    private float hOrientation = 0;
    private float xVelocity    = 0;
    private float yVelocity    = 0;
    private float hVelocity    = 0;

    private static final float goBILDA_SWINGARM_POD = 13.26291192f; //ticks-per-mm for the goBILDA Swingarm Pod
    private static final float goBILDA_4_BAR_POD    = 19.89436789f; //ticks-per-mm for the goBILDA 4-Bar Pod

    //i2c address of the device
    public static final byte DEFAULT_ADDRESS = 0x31;

    private EncoderDirection xEncoderDirection = EncoderDirection.FORWARD;
    private EncoderDirection yEncoderDirection = EncoderDirection.FORWARD;
    private double xOffset = 100;
    private double yOffset = 100;
    private double encoderResolution = goBILDA_4_BAR_POD;
    private double yawScalar = 1.0f;

    protected CommonOdometry odo;

    public GoBildaPinpointDriver(CommonOdometry odo){
        this.odo = odo;
    }

    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.Other;
    }

    @Override
    public String getDeviceName() {
        return "goBILDA® Pinpoint Odometry Computer";
    }

    //Device Status enum that captures the current fault condition of the device
    public enum DeviceStatus{
        NOT_READY                (0),
        READY                    (1),
        CALIBRATING              (1 << 1),
        FAULT_X_POD_NOT_DETECTED (1 << 2),
        FAULT_Y_POD_NOT_DETECTED (1 << 3),
        FAULT_NO_PODS_DETECTED   (1 << 2 | 1 << 3),
        FAULT_IMU_RUNAWAY        (1 << 4),
        FAULT_BAD_READ           (1 << 5);

        private final int status;

        DeviceStatus(int status){
            this.status = status;
        }
    }

    //enum that captures the direction the encoders are set to
    public enum EncoderDirection{
        FORWARD,
        REVERSED;
    }

    //enum that captures the kind of goBILDA odometry pods, if goBILDA pods are used
    public enum GoBildaOdometryPods {
        goBILDA_SWINGARM_POD,
        goBILDA_4_BAR_POD;
    }
    //enum that captures a limited scope of read data. More options may be added in future update
    public enum ReadData {
        ONLY_UPDATE_HEADING,
    }



    /**
     * Looks up the DeviceStatus enum corresponding with an int value
     * @param s int to lookup
     * @return the Odometry Computer state
     */
    private DeviceStatus lookupStatus (int s){
        if ((s & DeviceStatus.CALIBRATING.status) != 0){
            return DeviceStatus.CALIBRATING;
        }
        boolean xPodDetected = (s & DeviceStatus.FAULT_X_POD_NOT_DETECTED.status) == 0;
        boolean yPodDetected = (s & DeviceStatus.FAULT_Y_POD_NOT_DETECTED.status) == 0;

        if(!xPodDetected  && !yPodDetected){
            return DeviceStatus.FAULT_NO_PODS_DETECTED;
        }
        if (!xPodDetected){
            return DeviceStatus.FAULT_X_POD_NOT_DETECTED;
        }
        if (!yPodDetected){
            return DeviceStatus.FAULT_Y_POD_NOT_DETECTED;
        }
        if ((s & DeviceStatus.FAULT_IMU_RUNAWAY.status) != 0){
            return DeviceStatus.FAULT_IMU_RUNAWAY;
        }
        if ((s & DeviceStatus.READY.status) != 0){
            return DeviceStatus.READY;
        }
        if ((s & DeviceStatus.FAULT_BAD_READ.status) != 0){
            return DeviceStatus.FAULT_BAD_READ;
        }
        else {
            return DeviceStatus.NOT_READY;
        }
    }


    protected void updateEncoderValues(float oldX, float newX, float oldY, float newY, float oldH, float newH){
        float deltaX = newX - oldX;
        float deltaY = newY - oldY;
        float deltaH = newH - oldH;
        float avgH = oldH + deltaH / 2f;
        double cos = Math.cos(avgH);
        double sin = Math.sin(avgH);
        double deltaXR = deltaX * cos + deltaY * sin;
        double deltaYR = -deltaX * sin + deltaY * cos;
        xEncoderValue += encoderResolution * (deltaXR - xOffset * deltaH * (xEncoderDirection == EncoderDirection.FORWARD? 1f : -1f));
        yEncoderValue += encoderResolution * (deltaYR + yOffset * deltaH * (yEncoderDirection == EncoderDirection.FORWARD? 1f : -1f));
    }


    /**
     * Call this once per loop to read new data from the Odometry Computer. Data will only update once this is called.
     */
    protected synchronized void internalUpdate(boolean updateEncoders, boolean headingOnly){

        float oldPosX = xPosition;
        float oldPosY = yPosition;
        float oldPosH = hOrientation;

        CommonOdometry.PoseVelAccel pva = odo.getPoseVelAccel();

        float tempHeading = (float)pva.pos.getHeading(AngleUnit.RADIANS);
        float headingChange = AngleUnit.normalizeRadians(tempHeading - oldPosH);
        hOrientation = oldPosH + headingChange;

        if (!headingOnly) {
            xPosition = (float) pva.pos.getX(DistanceUnit.MM);
            yPosition = (float) pva.pos.getY(DistanceUnit.MM);

            xVelocity = (float) pva.vel.getX(DistanceUnit.MM);
            yVelocity = (float) pva.vel.getY(DistanceUnit.MM);
            hVelocity = (float) pva.vel.getHeading(AngleUnit.RADIANS);
        }

        if (updateEncoders) {
            updateEncoderValues(oldPosX, xPosition, oldPosY, yPosition, oldPosH, hOrientation);
        }

    }

    public synchronized void update(){
        internalUpdate(true, false);
    }


    /**
     * Call this once per loop to read new data from the Odometry Computer. This is an override of the update() function
     * which allows a narrower range of data to be read from the device for faster read times. Currently ONLY_UPDATE_HEADING
     * is supported.
     * @param data GoBildaPinpointDriver.ReadData.ONLY_UPDATE_HEADING
     */
    public synchronized void update(ReadData data) {
        if (data == ReadData.ONLY_UPDATE_HEADING){
            internalUpdate(true, true);
        }
    }

    /**
     * Sets the odometry pod positions relative to the point that the odometry computer tracks around.<br><br>
     * The most common tracking position is the center of the robot. <br> <br>
     * The X pod offset refers to how far sideways (in mm) from the tracking point the X (forward) odometry pod is. Left of the center is a positive number, right of center is a negative number. <br>
     * the Y pod offset refers to how far forwards (in mm) from the tracking point the Y (strafe) odometry pod is. forward of center is a positive number, backwards is a negative number.<br>
     * @param xOffset how sideways from the center of the robot is the X (forward) pod? Left increases
     * @param yOffset how far forward from the center of the robot is the Y (Strafe) pod? forward increases
     * @deprecated The overflow for this function has a DistanceUnit, which can reduce the chance of unit confusion.
     */
    public synchronized void setOffsets(double xOffset, double yOffset){
        this.xOffset = xOffset;
        this.yOffset = yOffset;
    }

    /**
     * Sets the odometry pod positions relative to the point that the odometry computer tracks around.<br><br>
     * The most common tracking position is the center of the robot. <br> <br>
     * The X pod offset refers to how far sideways from the tracking point the X (forward) odometry pod is. Left of the center is a positive number, right of center is a negative number. <br>
     * the Y pod offset refers to how far forwards from the tracking point the Y (strafe) odometry pod is. forward of center is a positive number, backwards is a negative number.<br>
     * @param xOffset how sideways from the center of the robot is the X (forward) pod? Left increases
     * @param yOffset how far forward from the center of the robot is the Y (Strafe) pod? forward increases
     * @param distanceUnit the unit of distance used for offsets.
     */
    public synchronized void setOffsets(double xOffset, double yOffset, DistanceUnit distanceUnit){
        setOffsets(distanceUnit.toMm(xOffset), distanceUnit.toMm(yOffset));
    }

    /**
     * Recalibrates the Odometry Computer's internal IMU. <br><br>
     * <strong> Robot MUST be stationary </strong> <br><br>
     * Device takes a large number of samples, and uses those as the gyroscope zero-offset. This takes approximately 0.25 seconds.
     */
    public void recalibrateIMU(){}

    /**
     * Resets the current position to 0,0,0 and recalibrates the Odometry Computer's internal IMU. <br><br>
     * <strong> Robot MUST be stationary </strong> <br><br>
     * Device takes a large number of samples, and uses those as the gyroscope zero-offset. This takes approximately 0.25 seconds.
     */
    public synchronized void resetPosAndIMU(){
        setPosition(new Pose2D(DistanceUnit.MM, 0, 0, AngleUnit.RADIANS, 0));
    }

    /**
     * Can reverse the direction of each encoder.
     * @param xEncoder FORWARD or REVERSED, X (forward) pod should increase when the robot is moving forward
     * @param yEncoder FORWARD or REVERSED, Y (strafe) pod should increase when the robot is moving left
     */
    public synchronized void setEncoderDirections(EncoderDirection xEncoder, EncoderDirection yEncoder){
        xEncoderDirection = xEncoder;
        yEncoderDirection = yEncoder;
    }

    /**
     * If you're using goBILDA odometry pods, the ticks-per-mm values are stored here for easy access.<br><br>
     * @param pods goBILDA_SWINGARM_POD or goBILDA_4_BAR_POD
     */
    public synchronized void setEncoderResolution(GoBildaOdometryPods pods){
        if (pods == GoBildaOdometryPods.goBILDA_SWINGARM_POD) {
            encoderResolution = goBILDA_SWINGARM_POD;
        }
        if (pods == GoBildaOdometryPods.goBILDA_4_BAR_POD){
            encoderResolution = goBILDA_4_BAR_POD;
        }
    }

    /**
     * Sets the encoder resolution in ticks per mm of the odometry pods. <br>
     * You can find this number by dividing the counts-per-revolution of your encoder by the circumference of the wheel.
     * @param ticks_per_mm should be somewhere between 10 ticks/mm and 100 ticks/mm a goBILDA Swingarm pod is ~13.26291192
     * @deprecated The overflow for this function has a DistanceUnit, which can reduce the chance of unit confusion.
     */
    public synchronized void setEncoderResolution(double ticks_per_mm){
        encoderResolution = ticks_per_mm;
    }

    /**
     * Sets the encoder resolution in ticks per mm of the odometry pods. <br>
     *
     * NOTE: the corresponding method in the original GoBilda driver has an error and won't work.
     *
     * You can find this number by dividing the counts-per-revolution of your encoder by the circumference of the wheel.
     * @param ticks_per_unit should be somewhere between 10 ticks/mm and 100 ticks/mm a goBILDA Swingarm pod is ~13.26291192
     * @param distanceUnit unit used for distance
     */
    public synchronized void setEncoderResolution(double ticks_per_unit, DistanceUnit distanceUnit){
        encoderResolution = 1.0 / distanceUnit.toMm(1.0 / ticks_per_unit);
//        encoderResolution = distanceUnit.toMm(ticks_per_unit);
    }

    /**
     * Tuning this value should be unnecessary.<br>
     * The goBILDA Odometry Computer has a per-device tuned yaw offset already applied when you receive it.<br><br>
     * This is a scalar that is applied to the gyro's yaw value. Increasing it will mean it will report more than one degree for every degree the sensor fusion algorithm measures. <br><br>
     * You can tune this variable by rotating the robot a large amount (10 full turns is a good starting place) and comparing the amount that the robot rotated to the amount measured.
     * Rotating the robot exactly 10 times should measure 3600°. If it measures more or less, divide moved amount by the measured amount and apply that value to the Yaw Offset.<br><br>
     * If you find that to get an accurate heading number you need to apply a scalar of more than 1.05, or less than 0.95, your device may be bad. Please reach out to tech@gobilda.com
     * @param yawOffset A scalar for the robot's heading.
     */
    public synchronized void setYawScalar(double yawOffset){
        yawScalar = yawOffset;
    }

    /**
     * Send a position that the Pinpoint should use to track your robot relative to. You can use this to
     * update the estimated position of your robot with new external sensor data, or to run a robot
     * in field coordinates. <br><br>
     * This overrides the current position. <br><br>
     * <strong>Using this feature to track your robot's position in field coordinates:</strong> <br>
     * When you start your code, send a Pose2D that describes the starting position on the field of your robot. <br>
     * Say you're on the red alliance, your robot is against the wall and closer to the audience side,
     * and the front of your robot is pointing towards the center of the field.
     * You can send a setPosition with something like -600mm x, -1200mm Y, and 90 degrees. The pinpoint would then always
     * keep track of how far away from the center of the field you are. <br><br>
     * <strong>Using this feature to update your position with additional sensors: </strong><br>
     * Some robots have a secondary way to locate their robot on the field. This is commonly
     * Apriltag localization in FTC, but it can also be something like a distance sensor.
     * Often these external sensors are absolute (meaning they measure something about the field)
     * so their data is very accurate. But they can be slower to read, or you may need to be in a very specific
     * position on the field to use them. In that case, spend most of your time relying on the Pinpoint
     * to determine your location. Then when you pull a new position from your secondary sensor,
     * send a setPosition command with the new position. The Pinpoint will then track your movement
     * relative to that new, more accurate position.
     * @param pos a Pose2D describing the robot's new position.
     */
    public synchronized Pose2D setPosition(Pose2D pos){
        xPosition = (float)pos.getX(DistanceUnit.MM);
        yPosition = (float)pos.getY(DistanceUnit.MM);
        hOrientation = (float)(pos.getHeading(AngleUnit.RADIANS));
        odo.setPosition(pos);
        return pos;
    }

    /**
     * Send a position that the Pinpoint should use to track your robot relative to.
     * You can use this to update the estimated position of your robot with new external
     * sensor data, or to run a robot in field coordinates.
     * @param posX the new X position you'd like the Pinpoint to track your robot relive to.
     * @param distanceUnit the unit for posX
     */
    public synchronized void setPosX(double posX, DistanceUnit distanceUnit){
        Pose2D pos = new Pose2D(DistanceUnit.MM, DistanceUnit.MM.fromUnit(distanceUnit, posX), yPosition, AngleUnit.RADIANS, hOrientation);
        setPosition(pos);
    }

    /**
     * Send a position that the Pinpoint should use to track your robot relative to.
     * You can use this to update the estimated position of your robot with new external
     * sensor data, or to run a robot in field coordinates.
     * @param posY the new Y position you'd like the Pinpoint to track your robot relive to.
     * @param distanceUnit the unit for posY
     */
    public synchronized void setPosY(double posY, DistanceUnit distanceUnit){
        Pose2D pos = new Pose2D(DistanceUnit.MM, xPosition, DistanceUnit.MM.fromUnit(distanceUnit, posY), AngleUnit.RADIANS, hOrientation);
        setPosition(pos);
    }

    /**
     * Send a heading that the Pinpoint should use to track your robot relative to.
     * You can use this to update the estimated position of your robot with new external
     * sensor data, or to run a robot in field coordinates.
     * @param heading the new heading you'd like the Pinpoint to track your robot relive to.
     * @param angleUnit Radians or Degrees
     */
    public synchronized void setHeading(double heading, AngleUnit angleUnit){
        Pose2D pos = new Pose2D(DistanceUnit.MM, xPosition, yPosition, angleUnit, heading);
    }

    /**
     * Checks the deviceID of the Odometry Computer. Should return 1.
     * @return 1 if device is functional.
     */
    public int getDeviceID(){return 1;}

    /**
     * @return the firmware version of the Odometry Computer
     */
    public int getDeviceVersion(){return 1;}

    /**
     * @return a scalar that the IMU measured heading is multiplied by. This is tuned for each unit
     * and should not need adjusted.
     */
    public synchronized float getYawScalar(){return (float)yawScalar; }

    /**
     * Device Status stores any faults the Odometry Computer may be experiencing. These faults include:
     * @return one of the following states:<br>
     * NOT_READY - The device is currently powering up. And has not initialized yet. RED LED<br>
     * READY - The device is currently functioning as normal. GREEN LED<br>
     * CALIBRATING - The device is currently recalibrating the gyro. RED LED<br>
     * FAULT_NO_PODS_DETECTED - the device does not detect any pods plugged in. PURPLE LED <br>
     * FAULT_X_POD_NOT_DETECTED - The device does not detect an X pod plugged in. BLUE LED <br>
     * FAULT_Y_POD_NOT_DETECTED - The device does not detect a Y pod plugged in. ORANGE LED <br>
     * FAULT_BAD_READ - The Java code has detected a bad I²C read, the result reported is a
     * duplicate of the last good read.
     */
    public synchronized DeviceStatus getDeviceStatus(){return lookupStatus(deviceStatus); }

    /**
     * Checks the Odometry Computer's most recent loop time.<br><br>
     * If values less than 500, or more than 1100 are commonly seen here, there may be something wrong with your device. Please reach out to tech@gobilda.com
     * @return loop time in microseconds (1/1,000,000 seconds)
     */
    public int getLoopTime(){return loopTime; }

    /**
     * Checks the Odometry Computer's most recent loop frequency.<br><br>
     * If values less than 900, or more than 2000 are commonly seen here, there may be something wrong with your device. Please reach out to tech@gobilda.com
     * @return Pinpoint Frequency in Hz (loops per second),
     */
    public double getFrequency(){
        if (loopTime != 0){
            return 1000000.0/loopTime;
        }
        else {
            return 0;
        }
    }

    /**
     * @return the raw value of the X (forward) encoder in ticks
     */
    public synchronized int getEncoderX(){return (int)Math.floor(xEncoderValue); }

    /**
     * @return the raw value of the Y (strafe) encoder in ticks
     */
    public synchronized int getEncoderY(){return (int)Math.floor(yEncoderValue); }

    /**
     * @return the estimated X (forward) position of the robot in mm
     * @deprecated The overflow for this function has a DistanceUnit, which can reduce the chance of unit confusion.
     */
    public synchronized double getPosX(){
        return xPosition;
    }

    /**
     * @return the estimated X (forward) position of the robot in specified unit
     * @param distanceUnit the unit that the estimated position will return in
     */
    public synchronized double getPosX(DistanceUnit distanceUnit){
        return distanceUnit.fromMm(xPosition);
    }

    /**
     * @return the estimated Y (Strafe) position of the robot in mm
     * @deprecated The overflow for this function has a DistanceUnit, which can reduce the chance of unit confusion.
     */
    public synchronized double getPosY(){
        return yPosition;
    }

    /**
     * @return the estimated Y (Strafe) position of the robot in specified unit
     * @param distanceUnit the unit that the estimated position will return in
     */
    public synchronized double getPosY(DistanceUnit distanceUnit){
        return distanceUnit.fromMm(yPosition);
    }

    /**
     * @return the unnormalized estimated H (heading) position of the robot in radians
     * unnormalized heading is not constrained from -180° to 180°. It will continue counting multiple rotations.
     * @deprecated two overflows for this function exist with AngleUnit parameter. These minimize the possibility of unit confusion.
     */
    public synchronized double getHeading(){
        return hOrientation;
    }


    /**
     * @return the estimated X (forward) velocity of the robot in mm/sec
     * @deprecated The overflow for this function has a DistanceUnit, which can reduce the chance of unit confusion.
     */
    public synchronized double getVelX(){
        return xVelocity;
    }

    /**
     * @return the estimated X (forward) velocity of the robot in specified unit/sec
     */
    public synchronized double getVelX(DistanceUnit distanceUnit){
        return distanceUnit.fromMm(xVelocity);
    }

    /**
     * @return the estimated Y (strafe) velocity of the robot in mm/sec
     * @deprecated The overflow for this function has a DistanceUnit, which can reduce the chance of unit confusion.
     */
    public synchronized double getVelY(){
        return yVelocity;
    }

    /**
     * @return the estimated Y (strafe) velocity of the robot in specified unit/sec
     */
    public synchronized double getVelY(DistanceUnit distanceUnit){
        return distanceUnit.fromMm(yVelocity);
    }

    /**
     * @return the estimated H (heading) velocity of the robot in radians/sec
     * @deprecated The overflow for this function has an AngleUnit, which can reduce the chance of unit confusion.
     */
    public synchronized double getHeadingVelocity() {
        return hVelocity;
    }

    /**
     * @return the estimated H (heading) velocity of the robot in specified unit/sec
     */
    public synchronized double getHeadingVelocity(UnnormalizedAngleUnit unnormalizedAngleUnit){
        return unnormalizedAngleUnit.fromRadians(hVelocity);
    }

    /**
     * <strong> This uses its own I2C read, avoid calling this every loop. </strong>
     * @return the user-set offset for the X (forward) pod in specified unit
     */
    public synchronized float getXOffset(DistanceUnit distanceUnit){
        return (float)distanceUnit.fromUnit(DistanceUnit.MM, xOffset);
    }

    /**
     * <strong> This uses its own I2C read, avoid calling this every loop. </strong>
     * @return the user-set offset for the Y (strafe) pod
     */
    public synchronized float getYOffset(DistanceUnit distanceUnit){
        return (float)distanceUnit.fromUnit(DistanceUnit.MM, yOffset);
    }

    /**
     * @return the unnormalized estimated H (heading) position of the robot in specified unit
     * unnormalized heading is not constrained from -180° to 180°. It will continue counting
     * multiple rotations.
     */
    public synchronized double getHeading(UnnormalizedAngleUnit unnormalizedAngleUnit){
        return unnormalizedAngleUnit.fromRadians(hOrientation);
    }

    /**
     * @return the normalized estimated H (heading) position of the robot in specified unit
     * normalized heading is wrapped from -180°, to 180°.
     *
     * NOTE: The corresponding method in the original GoBilda driver has an error and won't work.
     */
    public synchronized double getHeading(AngleUnit angleUnit){
        return angleUnit.fromRadians(((hOrientation + Math.PI) % (2 * Math.PI) + 2 * Math.PI) % (2 * Math.PI) - Math.PI);
    }

    /**
     * @return a Pose2D containing the estimated position of the robot
     */
    public synchronized Pose2D getPosition(){
        return new Pose2D(DistanceUnit.MM,
                xPosition,
                yPosition,
                AngleUnit.RADIANS,
                //this wraps the hOrientation variable from -180° to +180°
                ((hOrientation + Math.PI) % (2 * Math.PI) + 2 * Math.PI) % (2 * Math.PI) - Math.PI);
    }

    /**
     * @deprecated This function is not recommended, as velocity is wrapped from -180° to 180°.
     * instead use individual getters.
     * @return a Pose2D containing the estimated velocity of the robot, velocity is unit per second
     */
    public synchronized Pose2D getVelocity(){
        return new Pose2D(DistanceUnit.MM,
                xVelocity,
                yVelocity,
                AngleUnit.RADIANS,
                ((hVelocity + Math.PI) % (2 * Math.PI) + 2 * Math.PI) % (2 * Math.PI) - Math.PI);
    }
}





