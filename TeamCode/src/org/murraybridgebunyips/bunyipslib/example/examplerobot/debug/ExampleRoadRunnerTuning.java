package org.murraybridgebunyips.bunyipslib.example.examplerobot.debug;

import androidx.annotation.NonNull;

import org.murraybridgebunyips.bunyipslib.example.examplerobot.components.ExampleConfig;
import org.murraybridgebunyips.bunyipslib.roadrunner.drive.RoadRunnerDrive;
import org.murraybridgebunyips.bunyipslib.roadrunner.drive.TankRoadRunnerDrive;
import org.murraybridgebunyips.bunyipslib.roadrunner.drive.tuning.RoadRunnerTuning;

/**
 * Example of how to use the RoadRunner tuning clas.
 * You will simply need to extend RoadRunnerTuning and the selection for tuning procedure will be selected at runtime.
 * Note that these OpModes will not extend BunyipsOpMode, but uses a variant class which handles it for you.
 *
 * @noinspection UnnecessaryLocalVariable
 */
// It's more useful to annotate as TeleOp so the timer is off by default. You don't want to have an OpMode that stops
// all of a sudden because you forgot to turn off the Autonomous timer.
//@TeleOp(name = "RoadRunner Tuning")
public class ExampleRoadRunnerTuning extends RoadRunnerTuning {
    // While you can run the tuning OpModes yourself, it is not recommended. See the notes attached to RoadRunnerTuning.

    @NonNull
    @Override
    protected RoadRunnerDrive getBaseRoadRunnerDrive() {
        // Initialise your configuration as normal
        ExampleConfig config = new ExampleConfig();
        // Must pass `this` as a parameter as we are not in a BunyipsOpMode.
        config.init(this);

        // You can instantiate your drive here.

        // Do NOT use the BunyipsLib drive classes (MecanumDrive, TankDrive), you'll need to use the ones directly from RoadRunner (TankRoadRunnerDrive, MecanumRoadRunnerDrive)
        // This OpMode will refuse to work if you pass in the wrong drive class. This is to ensure the absolute base classes
        // from RoadRunner are working for you before BunyipsLib starts to abstract these systems.

        // You can choose to pass null for the telemetry field, it is not required (the OpMode will handle it for you)
        TankRoadRunnerDrive drive = new TankRoadRunnerDrive(
                null, config.driveConstants, config.coefficients, hardwareMap.voltageSensor,
                config.imu, config.leftFrontMotor, config.rightFrontMotor, config.leftBackMotor, config.rightBackMotor);

        // Set a localizer here if you have one (or it will use the default)
//        drive.setLocalizer(new TwoWheelTrackingLocalizer(ROBOT_CONFIG.localizerCoefficients, ROBOT_CONFIG.parallelEncoder, ROBOT_CONFIG.perpendicularEncoder, drive));

        // Allow the tuning OpMode to handle the rest
        return drive;
    }
}
