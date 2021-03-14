# A 2D simulator to help beginning Java programmers learn to program for FTC Robotics.

New: Programming Board configuration to serve as a companion to the book "Learn Java For FTC", by Alan Smith. The
PDF can be [downloaded for free](https://github.com/alan412/LearnJavaForFTC) or you can purchase the paperback on
[Amazon](https://www.amazon.com/dp/B08DBVKXLZ).

Also: Updated to allow use of either a full field or remote (8 x 12 ft) field for Ultimate Goal.

And: Addition of new Swerve robot configuration.
    
![](/readme_image.JPG)

This is a JavaFX application developed using the (free) IntelliJ IDEA Community Edition IDE. The repository can be downloaded
and unzipped, then opened with IntelliJ. It can also be run using Android Studio (see this [video](https://www.youtube.com/watch?v=pmaT9Twbmao)).

Six robot configurations are available: a simple two-wheeled robot, a robot with four mecanum wheels, an
X-Drive robot with four OmniWheels mounted at 45 degrees at each corner of the robot, a mecanum-wheeled 
configuration that has an extendable arm with a grabber at the end, a mecanum-wheeled configuration with
three "Dead-wheel" encoders for odometry, and a swerve-drive robot with four swerve units (each with a drive
motor, a cr-servo for steering, and an encoder to monitor steering).

Each robot can be thought of as 18 inches wide.  For the two-wheel bot and mecanum wheel bots, the distance between
the centers of the right and left wheels is 16 inches. For the mecanum wheel bots, the distance between the centers
of the front and back wheels is 14 inches, and the mecanum wheels (when viewed from the top) have an "X" configuration.
For the X-Drive bot, the distance between the centers of any two adjacent wheels is 14.5 inches. Each motor has an
encoder. There is a downward-facing color sensor in the center of the robot. A gyro sensor (or BNO055 imu) is also included.
The ArmBot has an extendable arm (DcMotor operated) with a grabber (Servo-operated) at the end. The other robots
have a simple rotating arm at the back. For the Mechanum bot and Two-Wheeled bot, the arm is controlled by a servo.
For the X-Drive bot, the arm is controlled by a CR servo. Each robot also has distance sensors on the front, left, right
and back sides. A small green rectangle indicates the front of the robot. Wheel diameters are all 4 inches. The bot 
with "Dead-wheel" encoders ("EncoderBot") has three dead-wheel encoders; the forward-reverse encoder wheels are 
mounted 6 inches to the right and left of center, while the X-direction (i.e., right-left) encoder wheel is
mounted at the center. The dead-wheels are two inches in diameter.

The field can be thought of as 12 feet wide. The field graphic (currently the Skystone field)
is obtained from a bitmap (.bmp) image. The color sensor detects the field color beneath the center of the
robot. The field graphic is easily changed by providing a different .bmp image in the virtual_robot.config.Config class.
The .bmp image is the skysone_field648.bmp file in the virtual_robot.assets folder. If a different .bmp image is used,
it must be at least as wide and as tall as the field dimensions (currently 648 x 648 pixels to fit on the screen of
most laptops). The Config class also allows selection between the use of "real" hardware gamepads versus a
"virtual gamepad".

In addition to the robot configurations described above, there is an additional configuration called
"ProgrammingBoard". It is meant to emulate the programming board described in the book "Learn Java For FTC", by
Alan Smith.  (The PDF can be [downloaded for free](https://github.com/alan412/LearnJavaForFTC) or you can purchase 
the paperback on [Amazon](https://www.amazon.com/dp/B08DBVKXLZ).)
It is a board with several hardware devices attached: DcMotor, Servo, Potentiometer, Touch Sensor,
and a Color-Distance Sensor. It also has a BNO055 IMU. The board doesn't move around the field, but it can
be rotated (to test the IMU) by dragging the board chassis.

An abridged approximation of the FTC SDK is provided.

User-defined OpModes must be placed in the org.firstinspires.ftc.teamcode package, and must extend OpMode (or LinearOpMode). OpModes are
registered by placing a @TeleOp or @Autonomous annotation immediately above the class declaration.

The OpMode (and therefore LinearOpMode) class in the simulator provides access to:

  1. A HardwareMap object, which in turn provides access to the DCMotor objects, the gyro sensor, distance sensors,
     the servo, and the color sensor;
  2. Two GamePads(actual hardware gamepads, though there is an option to use a "virtual gamepad -- see Log of Changes below");
  3. A Telemetry object.

An approximation of the FTC SDK's ElapsedTime class is provided in the time package.

Several example OpModes are provided in the org.firstinspires.ftc.teamcode package.

Some recent changes have simplified the process of creating new robot configurations (see  Log of Changes below).

To use:

  1. Make sure you have the Java 8 JDK installed on your PC. Also, install the free Community Edition of JetBrains
     IntelliJ IDEA.
  2. Download the virtual_robot .zip, and extract contents. Open the project in IntelliJ. You'll see three modules in
     the project (Controller, TeamCode, and virtual_robot) -- the only module you'll need to touch is TeamCode. It
     contains the org.firstinspires.ftc.teamcode package.
  3. Write your OpModes in the org.firstinspires.ftc.teamcode package; make sure to include a @TeleOp or @Autonomous annotation. These must
    extend the OpMode class (may either extend OpMode OR LinearOpMode). OpMode must provide init() and loop() methods;
     LinearOpMode must provide runOpMode() method.
  4. Make sure at least one gamepad is plugged in to the computer.
  5. Run the application (by clicking the green arrowhead at the toolbar).
  6. Press start-A or start-B on gamepad(s) to select which is gamepad1 vs. gamepad2.
  7. Use Configuration dropdown box to select a robot configuration. The configuration will be displayed.
  8. Use the Op Mode drop down box to select the desired OpMode.
  9. Prior to initialization, position the robot on the field by left-mouse-clicking the field (for robot position),
     and right-mouse-clicking (for robot orientation).
  10. Use the INIT/START/STOP button as you would on the FTC Driver Station.
  11. If desired use the sliders to introduce random and systematic motor error, and inertia.


LOG OF CHANGES

CHANGES 1/22/2021
    Changed telemetry to match (nearly completely) the API of the FTC SDK (but no speech). Also, add stack trace output 
    for exceptions thrown by op modes.

CHANGES 9/20/2020
    Added Swerve robot configuration. Each of four swerve units has: a DcMotor for drive, a CR-Servo for steering,
    and a separate encoder to monitor steering (this appears as a DcMotor in the config file). A TestSwerve op mode
    is included for demonstration.

CHANGES 9/13/2020
    Added the ability to "constrain" the field, to simulate partial fields being used for remote competitions. For a
    "RED" field, change the value of X_MIN_FRACTION in Config.java from 0 to 0.3333. For a partial "BLUE" field, change
    the value of X_MAX_FRACTION from 1 to 0.6667. This will mask the excluded parts of the field, and constrain robot
    motion. The distance sensors will behave as if the wall has been moved to the edge of the constraint area.

CHANGES 8/29/2020
    Added the ability to have the "virtual gamepad" triggers and joysticks "snap back" to zero when released. By
    default, they will hold at current position when released. But, if the SHIFT or ALT key is being pressed, then when
    these controls are released, they will return to zero. The default behavior can be changed by changing the value
    of HOLD_CONTROLS_BY_DEFAULT in virtual_robot.config.Config.java.

CHANGES 8/22/2020  
    Added programming board configuration to serve as a companion for the book "Learn Java For FTC", by Alan Smith.
   
The PDF can be [downloaded for free](https://github.com/alan412/LearnJavaForFTC) or you can purchase the paperback
on [Amazon](https://www.amazon.com/dp/B08DBVKXLZ).
                                                                                                   
CHANGES 7/22/2020
    Added "Dead-wheel" encoder capability, and a new robot configuration that has mecanum drive wheels and
    three dead-wheel encoders. Also added a new op mode to demonstrate dead-wheel odometry.

CHANGES 12/16/2019
    Further changes to facilitate creation of new robot configurations. The robot configuration classes (e.g., 
    MechanumBot) still extend VirtualBot. But now, these classes are also the JavaFX Controller classes for 
    the fxml markup files that define the robot's graphical representation in the UI. The robot configuration class 
    must have a @BotConfig annotation that indicates the name of this config (as it will be displayed to the user)
    and the filename of its corresponding fxml file. The fxml file must have a Group object as its root, and must
    set the fx:controller attribute of that group to the name of the robot config class. Individual nodes
    in the group can be given fx:id attributes, which make them accessible in the robot config class by using
    a @FXML annotation. The easiest way to create a new configuration is to copy, then modify, the ".java" and ".fxml"
    files from an existing configuration (for example, MechanumBot.java and mechanum_bot.fxml). See extensive comments
    in the virtual_robot.controller.VirtualBot and virtual_robot.controller.robots.classes.ArmBot classes and the
    virtual_robot.controller.robots.fxml.arm_bot.fxml file for more explanation.

CHANGES 12/12/2019
    Changes made to all more versatile building of new robot configurations. A transparent robot base layer (equal in
    width to the field) was added. This makes it possible for the robot to have accessories that extend well beyond
    the chassis in all four directions. A new robot configuration, ArmBot, was added. It has an extensible arm with a
    grabber at the end. The arm is DC Motor-operated. The grabber is Servo-operated. It is a mecanum-wheeled bot.

CHANGES 11/29/2019
    Range class and additional op modes contributed by FTC Team 16072. Servo interface (and ServoImpl class)
    enhanced with more features of the actual FTC SDK: ability to reverse direction and to scale position range.

CHANGES 10/6/2019
    Added the option of using "Virtual GamePad" instead of real GamePad. To do this, go to the Config.java class in the
    virtual_robot.config package (within the Controller module), and assign "true" to the USE_VIRTUAL_GAMEPAD constant.
    Other constants in this class include the field image (BACKGROUND) and the field width in pixels (FIELD_WIDTH). If
    changing FIELD_WIDTH, need to supply a square bitmap (.bmp) field image that is FIELD_WIDTH pixels wide.

CHANGES 8/17/2019
    RUN_TO_POSITION mode is now available for DcMotor, with setTargetPosition, getTargetPosition, and isBusy methods.
    Added 175 ms of latency to the BNO055IMU.

CHANGES 8/4/2019
    To better approximate real robot behavior, latency of 175ms added to the standard gyro sensor (used only on the
    Two-Wheel Bot). That is, updated values are available only every 175ms. The amount of latency can be changed
    easily in the createHardwareMap method of the virtual_robot.controller.robots.classes.TwoWheelBot class. Will probably make a
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
    you would in the FTC SDK. Two op modes for Mecanum Bot contributed by FTC team 16072, including a nice
    demonstration of field-centric drive using the IMU. These are in the org.firstinspires.ftc.teamcode.ftc16072 package.

CHANGES 6/25/2019
    Contribution from Alan Smith (alan412): now supports "regular" op modes in addition to linear op modes.

CHANGES 4/3/2019
    1. Added BNO055IMU interface to simulate (in a limited way) coding for the IMU in the REV Expansion Hub.
    2. The Mecanum Bot and X Drive Bot now have a BNO055IMU rather than the original gyro.
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

