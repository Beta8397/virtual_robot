This directory is reserved for example code to get started with the modified
BunyipsOpMode/BunyipsLib
ecosystem!

You will need to have knowledge with working with the FTC SDK, Android Studio, and Java/Kotlin to
use these examples effectively.

In a standard OpMode, we are used to using the `init()` and `loop()` methods to control our robot.
This is done by extending the `OpMode` class and overriding the `init()` and `loop()` methods.

# Similarity

BunyipsOpMode works similarly to the standard OpMode, but instead provides many more methods to
override and use. This is done by extending the `BunyipsOpMode` class and overriding the methods
you want to use. The only essential methods to override are `onInit()` and `activeLoop()`, which are
the equivalent of `init()` and `loop()` respectively.

# Ecosystem

The BunyipsOpMode ecosystem is a collection of classes and objects that are used to make writing
OpModes easier. The ecosystem is split into two parts: the `BunyipsLib` (previously `common`)
package and the `robot` package.
Each robot has its own `robot` package, and the `BunyipsLib` package is shared between all robots.
BunyipsLib is the current library that is used to write OpModes, and is a collection of classes and
objects that are used to make writing OpModes easier, with built in error handling, RoadRunner,
vision, command-based paradigms, and more.

View the BYOEcosystem.png image in this directory to view what methods are available to use in a
BunyipsOpMode, or look at specific implementations of OpModes such as AutonomousBunyipsOpMode,
CommandBasedBunyipsOpMode,
and RoadRunnerAutonomousBunyipsOpMode.

Each `robot` package contains classes and objects that are specific to that robot. This includes
their configuration, OpModes, and components specific to that robot.

# Configuration

All robots require a configuration file to be created in the `robot` package. This file is used to
map the hardware to the robot. This is done by extending the `RobotConfig` class and then using
this config in your OpMode. Functions from this differed way of initialisation includes built-in
error handling, ensuring your OpModes never crash due to a hardware error.

# Components

A major section of the BunyipsOpMode ecosystem is the theory of using Components. Components are
classes that are used to control a specific part of the robot, such as a motor or a sensor. These
components are then used in the OpMode to control the robot. Components are created by extending
`BunyipsSubsystem`, which will give that component access to the robot's overhead OpMode. You can
pass HardwareDevice types as constructor arguments to then make your component control that device.

Some components have common usage throughout robots, such as the preexisting IMUOp or Vision, which
is used to
control the IMU or camera systems. You're able to use these directly or make your own common
modules.
You will need to create components based on the different aspects of your robot.
(such as one for the arm, one for the drivetrain, one for the intake, etc.)

# Autonomous

Autonomous OpModes use a Task system, which is a set of smaller activeLoops that run at specific
times,
see the `autonomous` and `tasks` folder for some examples of what this looks like.

See the example robot and file structure in this directory, and copy what you need.

All examples are written in Java, but Kotlin examples exist if you look at the 'Jerry' robot
either in the archived folder or in the robot package.

The examples here are not exhaustive, you'll need to look at some of the BunyipsLib source code to
see
specific methods and their usage, or to look at older robots to see how they were implemented with
these development features. Documentation is a standard for all BunyipsLib source code.

# Making a new robot

In order to make a new robot, you'll need to make a new Gradle configuration for it, so you can
build and deploy robot specific code without having to group OpModes on different incompatible
robots.
By default, BunyipsLib will always be included with your builds, using the
build.common.gradle file in the root of the TeamCode directory.

You can make your own robot by copying the `../Template` directory and doing the following:

1. Renaming the directory name to your robot's name
2. Uncommenting the contents of build.gradle, and renaming the namespace to your robot name package
3. Editing the package namespace in src/main/AndroidManifest.xml
4. Editing the package name src/main/java/org/murraybridgebunyips/template to your robot name
5. Adding your robot to /settings.gradle
6. Removing the tmp file and populating the folder with your own code

Ensure to run a Gradle sync (Ctrl + Shift + O) after making these changes.

# Fast Load

Fast load is an experimental feature that builds and deploys your code to the robot without
rebuilding the entire project. This is useful for testing code quickly, but does not persist
between restarts of the Robot Controller. To fast load a robot configuration in Android Studio,
you will need to run the Gradle task `:<robotname>:reloadFastLoad`, substituting `<robotname>` for
your robot package.

To simplify this process, you can create an Android Studio run configuration that runs this task,
and ensure it saves
to the `/.run` directory for persistence across devices.

1. Open the Run/Debug Configurations window (Run > Edit Configurations...)
2. Click the + button in the top left corner and select Gradle
3. Name the configuration "`<robotname> (fast-load)`", substituting `<robotname>` for your robot
   package
4. Set the Gradle task (in Tasks and Arguments) to `:<robotname>:reloadFastLoad`
5. Ensure Gradle project is set to the root of the project (e.g BunyipsFTC)
6. Tick the box that says "Store as project file" and by default it should save to the `/.run`
   directory
7. Click OK to save the configuration

Ensure to install your TeamCode properly using the standard configuration when testing is complete.

###### Examples written by Lucas Bubner, 2023