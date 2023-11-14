package org.firstinspires.ftc.teamcode.example.examplerobot.components;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.common.RobotConfig;

/**
 * Example code for a robot configuration class under BunyipsOpMode ecosystem.
 */
// Extend the `RobotConfig` class when making a new config, as shown below.
public class ExampleConfig extends RobotConfig {
    // Here you will need to declare all of your instance variables, such as motors, servos, etc.
    // For example, if you have a motor called `leftMotor`, you will need to declare it like this:
    public DcMotor leftMotor;
    public DcMotor rightMotor;
    public DcMotor liftMotor;
    public WebcamName webcam;
    public IMU imu;
    // Put any other motors, servos, etc. here.
    // e.g. public Servo leftServo;
    //      public WebcamName webcam;

    // The init() function is required by RobotConfig to be implemented.
    // In the init function, you will initialise all your hardware with the handy getHardware()
    // function that is available for you to use.
    // This will map your instance variables (above) to the actual hardware you have named in
    // the Driver Station application.
    @Override
    protected void init() {
        // Find a Driver Station configuration name called 'left_motor' and assign it to the leftMotor instance variable
        // getHardware will return a type of HardwareDevice, and we can cast it to the type we want.
        // In this case, we want a DcMotor, so we cast it to a DcMotor.
        leftMotor = (DcMotor) getHardware("left_motor", DcMotor.class);
        // Repeat for all other hardware
        rightMotor = (DcMotor) getHardware("right_motor", DcMotor.class);
        liftMotor = (DcMotor) getHardware("lift_motor", DcMotor.class);
        webcam = (WebcamName) getHardware("webcam", WebcamName.class);

        // The reason we do it like this is that any errors will be caught and the program will not crash.
        // Additionally, telemetry will be sent to the Driver Station application to tell you what went wrong.

        // Perform any robot initialisation here, including IMU configuration, motor direction etc.
        // e.g leftMotor.setDirection(DcMotor.Direction.REVERSE);

        // Example IMU configuration
        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        );
        imu.initialize(parameters);
    }
}
