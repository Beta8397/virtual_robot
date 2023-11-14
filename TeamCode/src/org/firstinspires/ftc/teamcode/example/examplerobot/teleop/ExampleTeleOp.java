package org.firstinspires.ftc.teamcode.example.examplerobot.teleop;

import org.firstinspires.ftc.teamcode.common.BunyipsOpMode;
import org.firstinspires.ftc.teamcode.common.RobotConfig;
import org.firstinspires.ftc.teamcode.example.examplerobot.components.ExampleConfig;
import org.firstinspires.ftc.teamcode.example.examplerobot.components.ExampleDrive;

/**
 * Introduction to using BunyipsOpMode as the top-level component for a robot OpMode.
 */
// Here you would declare the OpMode to the driver station, e.g.
// @TeleOp(name="Example TeleOp")
public class ExampleTeleOp extends BunyipsOpMode {
    // Instead of extending OpMode or LinearOpMode, we extend BunyipsOpMode.
    // Here is where you will declare any components that you wish to use in your OpMode.
    // A required one is the config, which is used to map hardware to the variables you declare.

    // You MUST make a new instance of ExampleConfig as a class member
    // This is because the newConfig static method does not make an instance to work with
    private ExampleConfig config = new ExampleConfig();
    private ExampleDrive drive;

    // onInit and activeLoop are the two functions that you will need to implement.
    @Override
    protected void onInit() {
        // This line is required to initialise config and allow you to access all your instance
        // variables declared in the config class. This is required for all OpMode classes.
        config = (ExampleConfig) RobotConfig.newConfig(this, config, hardwareMap);

        // Initialise all your components! e.g.
        drive = new ExampleDrive(this, config.leftMotor, config.rightMotor);
    }

    @Override
    protected void activeLoop() {
        // You can access gamepad inputs using the built-in gamepad functions, e.g.
        // gamepad1.left_stick_x

        drive.run(gamepad1.left_stick_y);
        drive.update();

        // ActiveLoop will run every hardware cycle, therefore this code will continually set motor
        // power to the left stick y value.

        // You are also able to use telemetry by directly calling the built-in addTelemetry function
        // There is no need to call getOpMode() as we are calling from the OpMode.
        addTelemetry("Left Stick Y: " + gamepad1.left_stick_y);
    }

    // Additional methods can be used as well, see BYOEcosystem.png
    // e.g
    // @Override
    // protected void onInitDone() {}
}
