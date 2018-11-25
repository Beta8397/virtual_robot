A 2D simulator to help beginning Java programmers learn to program for FTC Robotics.

This is a JavaFX application developed using the IntelliJ IDEA Community Edition IDE. The repository can be downloaded
and unzipped, then opened with IntelliJ. I don't know how well it would work with other platforms, e.g. Eclipse.

The "virtual robot" can be thought of as 18 inches wide, with left and right drive motors, each attached to a
4 inch wheel. The distance between the centers of the two wheels can be thought of as 16 inches.Each motor has
an encoder. There is a downward-facing color sensor in the center of the robot. A gyro sensor is
also included. A purple slider on the back of the robot is controlled by a servo. A small green rectangle indicates the
front of the robot.

The field can be thought of as 12 feet wide. The field graphic (currently gray, with a pair of adjacent blue/red bands)
is obtained from a 600x600 pixel bitmap (.bmp) image. The color sensor detects the field color beneath the center of the
robot. The field graphic is easily changed by providing a different 600x600 .bmp image in the background.Background class.
The .bmp image is in the background.bmp file in the src/assets folder.

An abridged approximation of the FTC SDK is provided.

User-defined OpModes must be placed in the teamcode package, and must extend opmode.LinearOpMode. Each OpMode must be
registered by placing its name in the opModes list in the opmodelist.OpModes class.

The API for the simulator is documented with a javadoc.

The LinearOpMode class in the simulator provides access to:

  1. A HardwareMap object, which in turn provides access to the two DCMotor objects ("left_motor" and "right_motor"),
  the gyro sensor ("gyro_sensor"), the servo ("back_servo"), and the color sensor ("color_sensor");
  2. A GamePad (implemented in the UI, not a physical gamepad), with a subset of standard gamepad function;
  3. A Telemetry object.

An approximation of the FTC SDK's ElapsedTime class is provided in the time package.

Several example OpModes are provided in the teamcode package, and are already registered in the opModeList.OpModes class.

To use:

  1. Write your OpModes in the teamcode package, and register them in the opModeList.OpModes class. These must extend
  the LinearOpMode class, and must include a runOpMode method.
  2. Run the application.
  3. Use the drop down box to select the desired OpMode.
  4. Prior to initialization, position the robot on the field by left-mouse-clicking the field (for robot position),
   and right-mouse-clicking (for robot orientation).
  5. Use the INIT/START/STOP button as you would on the FTC Driver Station.

To use the virtual gamepad: mouse drag on the left and right joysticks; press the A,B,X,Y buttons.

