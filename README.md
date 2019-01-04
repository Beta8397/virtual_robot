A 2D simulator to help beginning Java programmers learn to program for FTC Robotics.

This is a JavaFX application developed using the (free) IntelliJ IDEA Community Edition IDE. The repository can be downloaded
and unzipped, then opened with IntelliJ.

Two robot configurations are available: a simple two-wheeled robot, and a robot with four mechanum wheels.
Each robot can be thought of as 18 inches wide. The distance between the centers of the right and left wheels
is 16 inches. For the mechanum wheel bot, the distance between the centers of the front and back wheels is 14 inches,
and the mechanum wheels (when viewed from the top) have an "X" configuration. Each motor has an encoder. There is a
downward-facing color sensor in the center of the robot. A gyro sensor is also included. A purple arm on the back
of the robot is controlled by a servo. A small green rectangle indicates the front of the robot.

The field can be thought of as 12 feet wide. The field graphic (currently gray, with a pair of adjacent blue/red bands)
is obtained from a bitmap (.bmp) image. The color sensor detects the field color beneath the center of the
robot. The field graphic is easily changed by providing a different .bmp image in the background.Background class.
The .bmp image is in the background.bmp file in the src/assets folder. If a different .bmp image is used,
it must be at least as wide and as tall as the field dimensions (currently 576 x 576 pixels to fit on the screen of
most laptops).

An abridged approximation of the FTC SDK is provided.

User-defined OpModes must be placed in the teamcode package, and must extend opmode.LinearOpMode. Each OpMode must be
registered by placing its name in the opModes list in the opmodelist.OpModes class. Note that this way of registering
OpModes differs from the @TeleOp and @Autonomous annotations of the FTC SDK.

The LinearOpMode class in the simulator provides access to:

  1. A HardwareMap object, which in turn provides access to the DCMotor objects, the gyro sensor,
     the servo, and the color sensor;
  2. A GamePad (implemented in the UI, not a physical gamepad), with a subset of standard gamepad function;
  3. A Telemetry object.

An approximation of the FTC SDK's ElapsedTime class is provided in the time package.

Several example OpModes are provided in the teamcode package, and are already registered in the opModeList.OpModes class.

To use:

  1. Make sure you have the Java JDK installed on your PC. Also, install the free Community Edition of JetBrains
     IntelliJ IDEA.
  2. Download the virtual_robot .zip, and extract contents. Open the project in IntelliJ. You'll see three modules in
     the project (Controller, TeamCode, and virtual_robot) -- the only module you'll need to touch is TeamCode. It
     contains the opmodelist and teamcode packages, as well as an assets directory.
  3. Write your OpModes in the teamcode package, and register them in the opModeList.OpModes class. These must extend
     the LinearOpMode class, and must include a runOpMode method.
  4. Run the application (by clicking the green arrowhead at the toolbar).
  5. Use Configuration dropdown box to select "Two Wheeled Bot" or "Mechanum Bot". The configuration will be displayed.
  6. Use the Op Mode drop down box to select the desired OpMode.
  7. Prior to initialization, position the robot on the field by left-mouse-clicking the field (for robot position),
     and right-mouse-clicking (for robot orientation).
  8. Use the INIT/START/STOP button as you would on the FTC Driver Station.
  9. To use the virtual gamepad: mouse drag on the left and right joysticks; press the A,B,X,Y buttons.
  10. If desired use the sliders to introduce random and systematic motor error.

