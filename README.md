A 2D simulator to help beginning Java programmers learn to program for FTC Robotics.

CHANGES 8/4/2019
    To better approximate real robot behavior, latency of 175ms added to the standard gyro sensor (used only on the
    Two-Wheel Bot). That is, updated values are available only every 175ms. The amount of latency can be changed
    easily in the createHardwareMap method of the virtual_robot.controller.TwoWheelBot class. Will probably make a
    similar change to the BNO055IMU soon.

CHANGES 7/10/2019
    To improve plug and play with OpModes copied and pasted from Android Studio, multiple packages were renamed. In
    addition, Continuous Rotation Servo capability was added. The XDrive Bot now has a CR Servo in the back rather
    than a standard servo. The XDriveBotDemo op mode demonstrates the use of this servo, using gamepad2.

    NOTE: OpModes copied directly from Android Studio to Virtual Robot do not automatically compile when pasted into
    Virtual Robot in IntelliJ, and won't show up in the OpModes dropdown box until they are compiled. Three different
    methods to force compilation are: 1) Right click the file and select "Recompile"; 2) From the "Build" menu,
    select "Rebuild Project"; or, 3) Make any change at all to the OpMode file (e.g., add a comment). Any one of these
    methods is sufficient.

CHANGES 7/06/2019
    Now uses @TeleOp, @Autonomous, and @Disabled class annotations to control the display of OpModes in the OpMode
    combobox. For @TeleOp and @Autonomous, a name parameter must be specified. The group parameter is optional (default
    group is "default"). GamePad setJoystickDeadzone capability contributed by FTC team 16072.

CHANGES 7/01/2019
    Now supports two GamePads instead of just one. Use start-A and start-B to select gamepad1 and gamepad2, as
    you would in the FTC SDK. Two op modes for Mechanum Bot contributed by FTC team 16072, including a nice
    demonstration of field-centric drive using the IMU. These are in the org.firstinspires.ftc.teamcode.ftc16072 package.

CHANGES 6/25/2019
    Contribution from Alan Smith (alan412): now supports "regular" op modes in addition to linear op modes.

CHANGES 4/3/2019
    1. Added BNO055IMU interface to simulate (in a limited way) coding for the IMU in the REV Expansion Hub.
    2. The Mechanum Bot and X Drive Bot now have a BNO055IMU rather than the original gyro.
    3. The Two-Wheel Bot still has the original gyro.
    4. DCMotor interface renamed to DcMotor, in keeping the the FTC SDK.
    5. New utility classes: enum AngleUnit, enum AxesOrder, enum AxesReference, class Orientation

CHANGES 3/23/2019
    1. Uses real game pad (instead of the original "virtual" game pad.
    2. Added an X-Drive robot configuration.
    3. Tweaks to opModeIsActive() and addition of isStopRequested() to allow while() loop before START.
    4. Added Color class with single static method: RGBtoHSV(red, green, blue, hsv).
    5. Added distance sensors to all robot configurations to measure distance from walls.
    6. Replaced LineFollow example opMode with MechBotAutoDemo, a line follower that actually works.

This is a JavaFX application developed using the (free) IntelliJ IDEA Community Edition IDE. The repository can be downloaded
and unzipped, then opened with IntelliJ.

Three robot configurations are available: a simple two-wheeled robot, a robot with four mechanum wheels, and an
X-Drive robot with four OmniWheels mounted at 45 degrees at each corner of the robot.

Each robot can be thought of as 18 inches wide.  For the two-wheel bot and mechanum wheel bot, the distance between
the centers of the right and left wheels is 16 inches. For the mechanum wheel bot, the distance between the centers
of the front and back wheels is 14 inches, and the mechanum wheels (when viewed from the top) have an "X" configuration.
For the X-Drive bot, the distance between the centers of any two adjacent wheels is 14.5 inches. Each motor has an
encoder. There is a downward-facing color sensor in the center of the robot. A gyro sensor is also included. A purple
arm on the back of the robot is controlled by a servo. Each robot also has distance sensors on the front, left, right
and back sides. A small green rectangle indicates the front of the robot. Wheel diameters are all 4 inches.

The field can be thought of as 12 feet wide. The field graphic (currently the Rover Ruckus field)
is obtained from a bitmap (.bmp) image. The color sensor detects the field color beneath the center of the
robot. The field graphic is easily changed by providing a different .bmp image in the background.Background class.
The .bmp image is in the background.bmp file in the src/virtual_robot.assets folder. If a different .bmp image is used,
it must be at least as wide and as tall as the field dimensions (currently 648 x 648 pixels to fit on the screen of
most laptops).

An abridged approximation of the FTC SDK is provided.

User-defined OpModes must be placed in the org.firstinspires.ftc.teamcode package, and must extend OpMode (or LinearOpMode). OpModes are
registered by placing a @TeleOp or @Autonomous annotation immediately above the class declaration.

The OpMode (and therefore LinearOpMode) class in the simulator provides access to:

  1. A HardwareMap object, which in turn provides access to the DCMotor objects, the gyro sensor, distance sensors,
     the servo, and the color sensor;
  2. Two GamePads(actual hardware gamepads);
  3. A Telemetry object.

An approximation of the FTC SDK's ElapsedTime class is provided in the time package.

Several example OpModes are provided in the org.firstinspires.ftc.teamcode package, and are already registered in the opModeList.OpModes class.

To use:

  1. Make sure you have the Java 8 JDK installed on your PC. Also, install the free Community Edition of JetBrains
     IntelliJ IDEA.
  2. Download the virtual_robot .zip, and extract contents. Open the project in IntelliJ. You'll see three modules in
     the project (Controller, TeamCode, and virtual_robot) -- the only module you'll need to touch is TeamCode. It
     contains the opmodelist and org.firstinspires.ftc.teamcode packages, as well as an virtual_robot.assets directory.
  3. Write your OpModes in the org.firstinspires.ftc.teamcode package; make sure to include a @TeleOp or @Autonomous annotation. These must
    extend the OpMode class (may either extend OpMode OR LinearOpMode). OpMode must provide init() and loop() methods;
     LinearOpMode must provide runOpMode() method.
  4. Make sure at least one gamepad is plugged in to the computer.
  5. Run the application (by clicking the green arrowhead at the toolbar).
  6. Press start-A or start-B on gamepad(s) to select which is gamepad1 vs. gamepad2.
  7. Use Configuration dropdown box to select "Two Wheeled Bot" or "Mechanum Bot". The configuration will be displayed.
  8. Use the Op Mode drop down box to select the desired OpMode.
  9. Prior to initialization, position the robot on the field by left-mouse-clicking the field (for robot position),
     and right-mouse-clicking (for robot orientation).
  10. Use the INIT/START/STOP button as you would on the FTC Driver Station.
  11. If desired use the sliders to introduce random and systematic motor error, and inertia.

