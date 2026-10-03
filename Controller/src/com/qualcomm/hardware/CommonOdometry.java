/*
    SPDX-License-Identifier: MIT

    Copyright (c) 2024 SparkFun Electronics
*/
/*
 * Modified by Team Beta 8397 for use in the virtual_robot simulator
 */
package com.qualcomm.hardware;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import virtual_robot.util.Vel2D;

/**
 *  INTERNAL USE ONLY
 *  Utility class for incorporating odometry sensors into virtual robot
 *  configurations
 */
public class CommonOdometry {

    private static CommonOdometry theInstance = null;

    private CommonOdometry(){}

    public static CommonOdometry getInstance(){
        if (theInstance == null) theInstance = new CommonOdometry();
        return theInstance;
    }

    /**
     * Internal use only
     */
    public static void internalReset(){
        theInstance = null;
    }

    /*
     * Save everything in METERS, RADIANS, and only convert to inches and/or
     * degrees when the user requests values.
     */

    // Raw pose in Meters, Radians
    protected Pose2D rawPoseMR = new Pose2D(DistanceUnit.METER,0,0, AngleUnit.RADIANS, 0);
    // Raw velocity in Meters/sec, Radians/sec
    protected Vel2D rawVelMR = new Vel2D(DistanceUnit.METER,0,0, UnnormalizedAngleUnit.RADIANS, 0);
    // Raw accelearation in Meters/sec2, Radians/sec2
    protected Vel2D rawAccelMR = new Vel2D(DistanceUnit.METER,0,0, UnnormalizedAngleUnit.RADIANS, 0);

    /*
     * The Base Pose is the pose of the User Coordinate System relative to the
     * Raw Coordinate System. The raw coordinate system for virtual_robot is X-Right,
     * Y-UP. The robot starts facing upward. Therefore, setting the base heading to
     * PI/2 results in a starting User Heading of 0. When the user calls setPosition,
     * a new instance of Pose2D is assigned to basePoseMR.
     */
    protected Pose2D basePoseMR = new Pose2D(DistanceUnit.METER,0, 0, AngleUnit.RADIANS, Math.PI/2);

    // position relative to basePoseMR, in Meters, Radians
    protected Pose2D positionMR = new Pose2D(DistanceUnit.METER,0,0, AngleUnit.RADIANS, 0);
    // velocity relative to basePoseMR (m/s, radians/s)
    protected Vel2D velocityMR = new Vel2D(DistanceUnit.METER,0,0, UnnormalizedAngleUnit.RADIANS, 0);
    // acceleration relative to basePoseMR (m/s2, radians/s)
    protected Vel2D accelMR = new Vel2D(DistanceUnit.METER,0,0, UnnormalizedAngleUnit.RADIANS, 0);


    /**
     * Update the CommonOdometry object to reflect current robot Pose. Parameters passed to this method are
     * in the RAW coordinate system (origin is center of field, X-Right, Y-Up), in MKS units.
     * @param posM    Raw pose in meters, radians
     * @param velM      Raw velocity in meters/sec, radians/sec
     * @param accelM    Raw acceleration in meters/sec2, radians/sec2
     */
    public synchronized void update(Pose2D posM, Vel2D velM, Vel2D accelM){
        rawPoseMR = posM;
        rawVelMR = velM;
        rawAccelMR = accelM;
        internalUpdate();
    }

    protected Pose2D rawPoseToPose(Pose2D rawPose){
        double cos = Math.cos(basePoseMR.getHeading(AngleUnit.RADIANS));
        double sin = Math.sin(basePoseMR.getHeading(AngleUnit.RADIANS));
        double poseX = (rawPose.getX(DistanceUnit.METER) - basePoseMR.getX(DistanceUnit.METER))*cos + (rawPose.getY(DistanceUnit.METER)-basePoseMR.getY(DistanceUnit.METER))*sin;
        double poseY = -(rawPose.getX(DistanceUnit.METER) - basePoseMR.getX(DistanceUnit.METER))*sin + (rawPose.getY(DistanceUnit.METER) - basePoseMR.getY(DistanceUnit.METER))*cos;
        return new Pose2D(DistanceUnit.METER, poseX, poseY, AngleUnit.RADIANS,
                AngleUnit.normalizeRadians(rawPose.getHeading(AngleUnit.RADIANS)-basePoseMR.getHeading(AngleUnit.RADIANS)+Math.PI/2.0));
    }

    protected Vel2D rawVelToVel(Vel2D rawVel, Pose2D rawPose){
//        double cos = Math.cos(rawPose.getHeading(AngleUnit.RADIANS));
//        double sin = Math.sin(rawPose.getHeading(AngleUnit.RADIANS));
//        double velX = -rawVel.getX(DistanceUnit.METER) * sin + rawVel.getY(DistanceUnit.METER) * cos;
//        double velY = -rawVel.getX(DistanceUnit.METER) * cos - rawVel.getY(DistanceUnit.METER) * sin;
//        return new Pose2D(DistanceUnit.METER, velX, velY, AngleUnit.RADIANS, rawVel.getHeading(AngleUnit.RADIANS));
        double cos = Math.cos(basePoseMR.getHeading(AngleUnit.RADIANS));
        double sin = Math.sin(basePoseMR.getHeading(AngleUnit.RADIANS));
        double velX = rawVel.getX(DistanceUnit.METER)*cos + rawVel.getY(DistanceUnit.METER)*sin;
        double velY = -rawVel.getX(DistanceUnit.METER)*sin + rawVel.getY(DistanceUnit.METER)*cos;
        return new Vel2D(DistanceUnit.METER, velX, velY, UnnormalizedAngleUnit.RADIANS, rawVel.getHeading(UnnormalizedAngleUnit.RADIANS));
    }

    protected Vel2D rawAccelToAccel(Vel2D rawAccel, Pose2D rawPose){
        return rawVelToVel(rawAccel, rawPose);
    }

    protected Pose2D poseToRawPose(Pose2D pose){
        double cos = Math.cos(basePoseMR.getHeading(AngleUnit.RADIANS));
        double sin = Math.sin(basePoseMR.getHeading(AngleUnit.RADIANS));
        double rawX = basePoseMR.getX(DistanceUnit.METER) + pose.getX(DistanceUnit.METER) * cos - pose.getY(DistanceUnit.METER) * sin;
        double rawY = basePoseMR.getY(DistanceUnit.METER) + pose.getX(DistanceUnit.METER) * sin + pose.getY(DistanceUnit.METER) * cos;
        return new Pose2D(DistanceUnit.METER, rawX, rawY, AngleUnit.RADIANS,
                AngleUnit.normalizeRadians(pose.getHeading(AngleUnit.RADIANS)+basePoseMR.getHeading(AngleUnit.RADIANS)-Math.PI/2.0));
    }

//    protected Pose2D velToRawVel(Pose2D vel){
//        double cos = Math.cos(basePoseMR.getHeading(AngleUnit.RADIANS));
//        double sin = Math.sin(basePoseMR.getHeading(AngleUnit.RADIANS));
//        double rawVX = vel.getX(DistanceUnit.METER) * cos - vel.getY(DistanceUnit.METER) * sin;
//        double rawVY = vel.getX(DistanceUnit.METER) * sin + vel.getY(DistanceUnit.METER) * cos;
//        return new Pose2D(DistanceUnit.METER, rawVX, rawVY, AngleUnit.RADIANS, vel.getHeading(AngleUnit.RADIANS));
//    }
//
//    protected Pose2D accelToRawAccel(Pose2D accel){
//        return velToRawVel(accel);
//    }

    protected synchronized void internalUpdate(){
        positionMR = rawPoseToPose(rawPoseMR);
        velocityMR = rawVelToVel(rawVelMR, rawPoseMR);
        accelMR = rawAccelToAccel(rawAccelMR, rawPoseMR);
    }

    /**
     * Gets the position measured by the CommonOdometry
     * @return Position measured by the CommonOdometry (in METERS, RADIANS)
     */
    public synchronized Pose2D getPosition() {
        Pose2D pos = new Pose2D(DistanceUnit.METER, positionMR.getX(DistanceUnit.METER), positionMR.getY(DistanceUnit.METER),
                AngleUnit.RADIANS, positionMR.getHeading(AngleUnit.RADIANS));
        return pos;
    }

    public synchronized Pose2D getPosition(DistanceUnit distanceUnit, AngleUnit angleUnit){
        Pose2D pos = new Pose2D(distanceUnit, positionMR.getX(distanceUnit), positionMR.getY(distanceUnit),
                angleUnit, positionMR.getHeading(angleUnit));
        return pos;
    }

    /**
     * Sets the position measured by the CommonOdometry. This is useful if your
     * robot does not start at the origin, or you have another source of
     * location information (eg. vision odometry); the CommonOdometry will continue
     * tracking from this position
     * @param pose New position for the CommonOdometry to track from
     */
    public synchronized void setPosition(Pose2D pose) {
        // Convert pose to meters and radians.
        pose = new Pose2D(DistanceUnit.METER, pose.getX(DistanceUnit.METER), pose.getY(DistanceUnit.METER),
                AngleUnit.RADIANS, pose.getHeading(AngleUnit.RADIANS));
        double hBase = AngleUnit.normalizeRadians(rawPoseMR.getHeading(AngleUnit.RADIANS) - pose.getHeading(AngleUnit.RADIANS) + Math.PI/2.0);
        double cos = Math.cos(hBase);
        double sin = Math.sin(hBase);
        double xBase = rawPoseMR.getX(DistanceUnit.METER) - pose.getX(DistanceUnit.METER) * cos + pose.getY(DistanceUnit.METER) * sin;
        double yBase = rawPoseMR.getY(DistanceUnit.METER) - pose.getX(DistanceUnit.METER) * sin - pose.getY(DistanceUnit.METER) * cos;
        basePoseMR = new Pose2D(DistanceUnit.METER, xBase, yBase, AngleUnit.RADIANS, hBase);
        internalUpdate();
    }

    /**
     * Gets the velocity measured by the CommonOdometry
     * @return Velocity measured by the CommonOdometry
     */
    public synchronized Vel2D getVelocity() {
//        Vel2D vel = new Vel2D(DistanceUnit.METER, velocityMR.getX(DistanceUnit.METER), velocityMR.getY(DistanceUnit.METER),
//                UnnormalizedAngleUnit.RADIANS, velocityMR.getHeading(UnnormalizedAngleUnit.RADIANS));
//        return vel;
        return velocityMR;
    }

//    public synchronized Vel2D getVelocity(DistanceUnit distanceUnit, UnnormalizedAngleUnit angleUnit) {
//
//        Vel2D vel = new Vel2D(distanceUnit, velocityMR.getX(distanceUnit), velocityMR.getY(distanceUnit),
//                angleUnit, velocityMR.getHeading(UnnormalizedAngleUnit.RADIANS));
//        return vel;
//    }

    /**
     * Gets the acceleration measured by the CommonOdometry
     * @return Acceleration measured by the CommonOdometry
     */
    public synchronized Vel2D getAcceleration() {
//        Vel2D acc = new Vel2D(DistanceUnit.METER, accelMR.getX(DistanceUnit.METER), accelMR.getY(DistanceUnit.METER),
//                UnnormalizedAngleUnit.RADIANS, accelMR.getHeading(UnnormalizedAngleUnit.RADIANS));
//        return acc;
        return accelMR;
    }

//    public synchronized Vel2D getAcceleration(DistanceUnit distanceUnit, UnnormalizedAngleUnit angleUnit){
//        Vel2D acc = new Vel2D(distanceUnit, accelMR.getX(distanceUnit), accelMR.getY(distanceUnit),
//                angleUnit, accelMR.getHeading(angleUnit));
//        return acc;
//    }

    /**
     *  Utility class to hold a position, velocity and acceleration
     */
    public class PoseVelAccel {

        public final Pose2D pos;
        public final Vel2D vel;
        public final Vel2D acc;

        public PoseVelAccel(Pose2D pose, Vel2D velocity, Vel2D acceleration){
            pos = pose;
            vel = velocity;
            acc = acceleration;
        }

    }

    public synchronized PoseVelAccel getPoseVelAccel(){
        return new PoseVelAccel(getPosition(), getVelocity(), getAcceleration());
    }

//    public synchronized PoseVelAccel getPoseVelAccel(DistanceUnit distanceUnit, AngleUnit angleUnit){
//        UnnormalizedAngleUnit unnormalizedAngleUnit = angleUnit.getUnnormalized();
//        return new PoseVelAccel(getPosition(distanceUnit, angleUnit), getVelocity(distanceUnit, unnormalizedAngleUnit),
//                getAcceleration(distanceUnit, unnormalizedAngleUnit));
//    }

}
