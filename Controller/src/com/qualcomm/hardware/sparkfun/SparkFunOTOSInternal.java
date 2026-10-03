package com.qualcomm.hardware.sparkfun;

import com.qualcomm.hardware.CommonOdometry;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

/**
 *   SparkFunOTOS methods for internal use only
 */
public class SparkFunOTOSInternal extends SparkFunOTOS{

    /**
     * Internal use only: update position, velocity, and acceleration from CommonOdometry
     */

    public void update(){
        CommonOdometry.PoseVelAccel pvaMR = odo.getPoseVelAccel();
        UnnormalizedAngleUnit unnormalizedAngleUnit = _angularUnit.getUnnormalized();

        position = new Pose2D(pvaMR.pos.getX(_distanceUnit), pvaMR.pos.getY(_distanceUnit), pvaMR.pos.getHeading(_angularUnit));
        velocity = new Pose2D(pvaMR.vel.getX(_distanceUnit), pvaMR.vel.getY(_distanceUnit), pvaMR.vel.getHeading(unnormalizedAngleUnit));
        acceleration = new Pose2D(pvaMR.acc.getX(_distanceUnit), pvaMR.acc.getY(_distanceUnit), pvaMR.acc.getHeading(unnormalizedAngleUnit));
    }

}
