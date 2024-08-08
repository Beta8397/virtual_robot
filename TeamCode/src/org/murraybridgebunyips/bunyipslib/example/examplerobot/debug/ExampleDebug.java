package org.murraybridgebunyips.bunyipslib.example.examplerobot.debug;

import org.murraybridgebunyips.bunyipslib.BunyipsOpMode;
import org.murraybridgebunyips.bunyipslib.example.examplerobot.components.ExampleConfig;
import org.murraybridgebunyips.bunyipslib.subsystems.IMUOp;

/**
 * Example debugging code for a robot OpMode under BunyipsOpMode ecosystem.
 */
public class ExampleDebug extends BunyipsOpMode {
    // Debug OpModes under the 'debug' directory are used to test components and tasks!
    // These aren't used in competition, but are useful for testing code.
    // This file is an example test to ensure that the IMU is working correctly, by printing the
    // heading constantly to the Driver Station

    private final ExampleConfig config = new ExampleConfig();
    // All components and tasks must be instantiated during runtime, and not in the constructor or member fields.
    private IMUOp imu;

    @Override
    protected void onInit() {
        config.init();
        imu = new IMUOp(config.imu);
    }

    @Override
    protected void activeLoop() {
        telemetry.add("Heading: " + imu.yaw);
        // Alternatively the custom format string method, where type can be omitted: telemetry.add("Heading: %", imu.getHeading());

        imu.update();
    }
}
