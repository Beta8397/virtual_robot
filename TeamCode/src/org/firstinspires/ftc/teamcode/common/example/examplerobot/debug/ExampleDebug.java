package org.firstinspires.ftc.teamcode.common.example.examplerobot.debug;

import org.firstinspires.ftc.teamcode.common.BunyipsOpMode;
import org.firstinspires.ftc.teamcode.common.IMUOp;
import org.firstinspires.ftc.teamcode.common.example.examplerobot.components.ExampleConfig;

/**
 * Example debugging code for a robot OpMode under BunyipsOpMode ecosystem.
 */
public class ExampleDebug extends BunyipsOpMode {
    // Debug OpModes under the 'debug' directory are used to test components and tasks!
    // These aren't used in competition, but are useful for testing code.
    // This file is an example test to ensure that the IMU is working correctly, by printing the
    // heading constantly to the Driver Station

    private final ExampleConfig config = new ExampleConfig();
    private IMUOp imu;

    @Override
    protected void onInit() {
        config.init(this);
        imu = new IMUOp(this, config.imu);
    }

    @Override
    protected void activeLoop() {
        addTelemetry("Heading: " + imu.getHeading(), false);
    }
}
