# A 2D simulator to help beginning Java programmers learn to program for FTC Robotics.

New: 

Now supports RoadRunner v1.0.1 AND PedroPathing v2.0.4, including facsimiles of the 
Quickstart teamcode for each. For both RR and PP, tuning has been done for 
virtual_robot's MecDynamic robot configuration. This is the most physically realistic 
robot configuration; the other configurations aren't expected to work well 
with RR or PP. Example opmodes are included.

Two new items (DEFAULT_DRIVE_MOTOR_TYPE and DEFAULT_BOT) are added to the Config.java class. 
You can change these if you'd prefer that your favorite robot configuration use a different drive 
motor type, or if you would prefer to have a different robot configuration appear when you first 
run the app.

If you want to continue using PedroPathing 1, you can use git to clone the repository, 
then reset to commit #eca722d, as follows:

git reset --hard eca722d

Current game configuration is Decode. This includes goals and ramps, which serve as obstacles to robot
travel. If you don't want these obstacles, then in the Config.java file, change the 
assignment statement for GAME to: new NoGame()

GoBilda Pinpoint odometry sensor added. It is included in Mecanum and XDrive robots. ("pinpoint" in config file)

SparFunOTOS Odometry Sensor added. It is included in Mecanum and XDrive robots. ("sensor_otos" in config file)

OctoQuad Encoder Sensor added. It is included in the Mecanum and XDrive robot configurations, with channels
assigned as follows: 0->back left drive motor; 1->front left drive motor; 2->front right drive motor;
3->back right motor; 4->left deadwheel encoder; 5->right deadwheel encoder; 6-> perpendicular deadwheel
encoder. You can still use the traditional way of working with the encoders, as well.

Includes Programming Board configuration to serve as a companion to the book "Learn Java For FTC", by Alan Smith. The
PDF can be [downloaded for free](https://github.com/alan412/LearnJavaForFTC) or you can purchase the paperback on
[Amazon](https://www.amazon.com/dp/B08DBVKXLZ).
    
![](/readme_gif.gif)

This is a JavaFX application developed using the (free) IntelliJ IDEA Community Edition IDE. The repository can be downloaded
and unzipped (or cloned with git), then opened with IntelliJ or Android Studio.

Multiple robot configurations are available in the "Configurations" dropdown menu. Several other configurations 
(SwerveBot, DiffSwerveBot, etc) are available, but are currently disabled. They can be enabled by un-commenting 
the @Botconfig annotations in their configuration classes (package virtual_robot.robots.classes).

Each robot can be thought of as 18 inches wide.  There is a downward-facing color sensor in the center of each robot. 
A BNO055 IMU is also included. Each robot also has distance sensors on the front, left, right and back sides. 
A small green rectangle indicates the front of each robot. Wheel diameters are all 4 inches. For the robots with 
dead-wheel encoders (MecanumBot and XDriveBot), the forward-reverse encoder wheels are mounted 6 inches to the right and 
left of center, while the X-direction (i.e., right-left) encoder wheel is mounted at the center. The dead-wheels are 
two inches in diameter. Positioning of the dead-wheels can be changed easily in the robot configuration classes.

The field can be thought of as 12 feet wide. The field graphic (currently the Freight Frenzy field)
is obtained from a bitmap (.bmp) image. The color sensor detects the field color beneath the center of the
robot. The field graphic is easily changed by providing a different .bmp image in the virtual_robot.config.Config class.
The .bmp image is the freight_field648.bmp file in the virtual_robot.assets folder. If a different .bmp image is used,
it must be at least as wide and as tall as the field dimensions (currently 648 x 648 pixels to fit on the screen of
most laptops). The Config class also allows selection between the use of "real" hardware gamepads versus a
"virtual gamepad".

In addition to the robot configurations described above, there is an additional configuration called
"ProgrammingBoard". It is meant to emulate the programming board described in the book "Learn Java For FTC", by
Alan Smith.  (The PDF can be [downloaded for free](https://github.com/alan412/LearnJavaForFTC) or you can purchase 
the paperback on [Amazon](https://www.amazon.com/dp/B08DBVKXLZ).) It is a board with several hardware devices 
attached: DcMotor, Servo, Potentiometer, Touch Sensor, and a Color-Distance Sensor. It also has a BNO055 IMU. 
The board doesn't move around the field, but it can be rotated (to test the IMU) by dragging the board chassis.

An approximation of the FTC SDK is provided.

User-defined OpModes must be placed in the org.firstinspires.ftc.teamcode package, and must extend OpMode 
(or LinearOpMode). OpModes are registered by placing a @TeleOp or @Autonomous annotation immediately above the class 
declaration.

Several example OpModes are provided in the org.firstinspires.ftc.teamcode package. To minimize clutter, a number 
of sample op modes are currently disabled; they can be re-enabled by commenting out the @Disabled annotation. A 
number of robot configurations are also disabled. A robot configuration can be re-enabled by finding its class 
in the virtual_robot.robots.classes package, and un-commenting its @BotConfig annotation.

To use:

  1. Install IntelliJ IDEA (or Android Studio), the Liberica 17 JDK (FULL VERSION!!), and the virtual_robot project,
     per the "Detailed Installation Instructions" PDF.
  3. Write your OpModes in the org.firstinspires.ftc.teamcode package; make sure to include a @TeleOp or @Autonomous 
     annotation. These must extend the OpMode class (may either extend OpMode OR LinearOpMode). OpMode must provide 
     init() and loop() methods; LinearOpMode must provide runOpMode() method.
  4. You can either use the Virtual Gamepad (currently the default), or use one or two real Gamepads; to use real 
     gamepads, open Controller/src/virtual_robot/config/Config.java and set USE_VIRTUAL_GAMEPAD to false. Then plug 
     in your gamepad(s). 
  5. Run the application (by clicking the green arrowhead at the toolbar).
  6. If using real gamepad(s), press start-A or start-B on gamepad(s) to select which is gamepad1 vs. gamepad2.
  7. Use Configuration dropdown box to select a robot configuration. The configuration file will be displayed.
  8. Use the Op Mode drop down box to select the desired OpMode.
  9. Prior to initialization, position the robot on the field by left-mouse-clicking the field (for robot position),
     and right-mouse-clicking (for robot orientation).
  10. Use the INIT/START/STOP button as you would on the FTC Driver Station.
  11. If desired use the sliders to introduce random and systematic motor error, and inertia.

    NOTE: OpModes copied directly from Android Studio to Virtual Robot may not automatically compile when pasted into
    Virtual Robot in IntelliJ, and won't show up in the OpModes dropdown box until they are compiled. Two different
    methods to force compilation are: 1) From the "Build" menu, select "Rebuild Project"; or, 2) Make any change at all 
    to the OpMode file (e.g., add a comment). Any one of these
    methods is sufficient.

Acknowledgement: Thanks to Alan Smith for a contribution in 2019 for a contribution that provides compatibility with
a wider variety of OpModes ("iterative" opmodes in addition to the original "linear" opmode support).

