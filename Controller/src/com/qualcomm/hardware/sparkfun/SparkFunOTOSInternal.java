package com.qualcomm.hardware.sparkfun;

import com.qualcomm.hardware.CommonOdometry;

/**
 *   SparkFunOTOS methods for internal use only
 */
public class SparkFunOTOSInternal extends SparkFunOTOS{

    public SparkFunOTOSInternal(CommonOdometry odo){
        super(odo);
    }


    /**
     * Internal use only: update position, velocity, and acceleration from CommonOdometry
     */

    public void update(){
        CommonOdometry.PoseVelAccel pva = odo.getPoseVelAccel(_distanceUnit, _angularUnit);
        position = new Pose2D(pva.pos.getX(_distanceUnit), pva.pos.getY(_distanceUnit), pva.pos.getHeading(_angularUnit));
        velocity = new Pose2D(pva.vel.getX(_distanceUnit), pva.vel.getY(_distanceUnit), pva.vel.getHeading(_angularUnit));
        acceleration = new Pose2D(pva.acc.getX(_distanceUnit), pva.acc.getY(_distanceUnit), pva.acc.getHeading(_angularUnit));
    }

}
