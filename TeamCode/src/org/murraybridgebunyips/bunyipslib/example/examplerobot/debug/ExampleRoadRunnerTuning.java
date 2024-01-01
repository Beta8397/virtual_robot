package org.murraybridgebunyips.bunyipslib.example.examplerobot.debug;

import org.murraybridgebunyips.bunyipslib.example.examplerobot.components.ExampleConfig;
import org.murraybridgebunyips.bunyipslib.roadrunner.drive.TankRoadRunnerDrive;
import org.murraybridgebunyips.bunyipslib.roadrunner.drive.tuning.ManualFeedforwardTuner;

/**
 * Example of how to use the RoadRunner tuning classes.
 * You will need to extend whichever tuning class you are using, and pass through your config.
 * Note that these OpModes will not extend BunyipsOpMode.
 */
//@Autonomous(name = "RoadRunner Tuning")
public class ExampleRoadRunnerTuning extends ManualFeedforwardTuner { // See all tuning opmodes in BunyipsLib/roadrunner/drive/tuning
    @Override
    public void runOpMode() {
        // Initialise your configuration as normal
        ExampleConfig ROBOT_CONFIG = new ExampleConfig();
        ROBOT_CONFIG.init(this);

        // drive is defined in the superclass, assign it here to a new instance of your drive class
        // Do not use the BunyipsLib drive classes (MecanumDrive, TankDrive), you'll need to use the ones directly from RoadRunner (TankRoadRunnerDrive, MecanumRoadRunnerDrive)
        drive = new TankRoadRunnerDrive(ROBOT_CONFIG.driveConstants, ROBOT_CONFIG.coefficients, hardwareMap.voltageSensor, ROBOT_CONFIG.imu, ROBOT_CONFIG.leftFrontMotor, ROBOT_CONFIG.rightFrontMotor, ROBOT_CONFIG.leftBackMotor, ROBOT_CONFIG.rightBackMotor);
//        drive.setLocalizer(new TwoWheelTrackingLocalizer(ROBOT_CONFIG.localizerCoefficients, ROBOT_CONFIG.parallelEncoder, ROBOT_CONFIG.perpendicularEncoder, drive));

        // Allow the tuning OpMode to handle the rest
        super.runOpMode();
    }
}
