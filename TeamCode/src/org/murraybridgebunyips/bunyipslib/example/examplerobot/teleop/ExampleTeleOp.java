package org.murraybridgebunyips.bunyipslib.example.examplerobot.teleop;

import org.murraybridgebunyips.bunyipslib.BunyipsOpMode;
import org.murraybridgebunyips.bunyipslib.Controls;
import org.murraybridgebunyips.bunyipslib.drive.TankDrive;
import org.murraybridgebunyips.bunyipslib.example.examplerobot.components.ExampleConfig;

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
    private final ExampleConfig config = new ExampleConfig();
    // All components and tasks must be instantiated during runtime, and not in the constructor or member fields.
    private TankDrive drive;

    // onInit and activeLoop are the two functions that you will need to implement.
    @Override
    protected void onInit() {
        // This line is required to initialise config and allow you to access all your instance
        // variables declared in the config class. This is required for all OpMode classes.
        config.init();

        // Initialise all your components! e.g.
        drive = new TankDrive(config.driveConstants, config.coefficients, config.imu, config.leftFrontMotor, config.leftBackMotor, config.leftBackMotor, config.rightBackMotor);
    }

    @Override
    protected void activeLoop() {
        // You can access gamepad inputs using the built-in gamepad functions, e.g.
        // gamepad1.left_stick_x

        drive.setWeightedDrivePower(Controls.makeRobotPose(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x));
        drive.update();

        // ActiveLoop will run every hardware cycle, therefore this code will continually set motor
        // power to the left stick y value.

        // You are also able to use telemetry by directly calling the built-in addTelemetry function
        // There is no need to call opMode as we are calling from the OpMode.
        telemetry.add("Left Stick Y: " + gamepad1.left_stick_y);
    }

    // Additional methods can be used as well, see BYOEcosystem.png
    // e.g
    // @Override
    // protected void onInitDone() {}
}
