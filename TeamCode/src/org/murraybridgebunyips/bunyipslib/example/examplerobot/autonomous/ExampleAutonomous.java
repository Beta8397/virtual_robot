package org.murraybridgebunyips.bunyipslib.example.examplerobot.autonomous;

import static org.murraybridgebunyips.bunyipslib.external.units.Units.Seconds;

import androidx.annotation.Nullable;

import org.murraybridgebunyips.bunyipslib.AutonomousBunyipsOpMode;
import org.murraybridgebunyips.bunyipslib.Controls;
import org.murraybridgebunyips.bunyipslib.Reference;
import org.murraybridgebunyips.bunyipslib.StartingPositions;
import org.murraybridgebunyips.bunyipslib.drive.CartesianMecanumDrive;
import org.murraybridgebunyips.bunyipslib.example.examplerobot.components.ExampleConfig;
import org.murraybridgebunyips.bunyipslib.example.examplerobot.components.ExampleLift;
import org.murraybridgebunyips.bunyipslib.example.examplerobot.tasks.ExampleTask;
import org.murraybridgebunyips.bunyipslib.tasks.WaitTask;

/**
 * Example autonomous OpMode for a robot with a lift and a mecanum drive.
 */
public class ExampleAutonomous extends AutonomousBunyipsOpMode {
    // This class extends BunyipsOpMode, which is the base class for all OpModes in BunyipsLib
    // If you need to use RoadRunner, it is recommended to implement the RoadRunner interface as well,
    // which is an interface that allows for RoadRunner methods to be used with special
    // methods such as makeTrajectory()

    // The primary difference between AutonomousBunyipsOpMode and BunyipsOpMode is that looping
    // is handled differently. In BunyipsOpMode, the activeLoop() method is called repeatedly, and you
    // update state in that method (e.g. checking for button presses to update hardware). In AutonomousBunyipsOpMode,
    // a queue of tasks is internally managed, and the OpMode moves through running these smaller
    // tasks that consist of their own activeLoop() methods. This allows for more complex autonomous
    // programs to be written, allowing a linear sequence of tasks to be run despite the iterative
    // nature of OpModes.

    // When you instantiate a new AutonomousBunyipsOpMode, many methods will be overridden from BunyipsOpMode,
    // check the definition of AutonomousBunyipsOpMode for more information and to see if you need to exercise
    // caution when overriding custom methods. Most cases you will be fine to use the abstract methods of
    // AutonomousBunyipsOpMode, but if you need to override a method, check the definition first.

    // You will define your components as normal here, and initialise them in onInitialise()
    // This procedure is exactly the same as a normal BunyipsOpMode
    private final ExampleConfig config = new ExampleConfig();
    // All components and tasks must be instantiated during runtime, and not in the constructor or member fields.
    private CartesianMecanumDrive drive;
    private ExampleLift lift;

    @Override
    protected void onInitialise() {
        config.init();
        drive = new CartesianMecanumDrive(config.leftFrontMotor, config.rightFrontMotor, config.leftBackMotor, config.rightBackMotor);
        lift = new ExampleLift(config.liftMotor);

        // setOpModes is used to define dynamic OpMode selection, using an asynchronous UserSelection
        // This is not required, and you can return null if you do not need to use this feature
        // However, if you do need to use this feature, you must return a list of OpModeSelections
        // which will be automatically handled to show up as gamepad inputs when you initialise the OpMode
        // When one of these inputs is pressed, the OpMode will return the corresponding OpModeSelection
        // to the onReady() method, which you can use to determine which OpMode to run.
        setOpModes(StartingPositions.use());
    }

    @Override
    protected void onReady(@Nullable Reference<?> selectedOpMode, Controls selectedButton) {
        // onReady will return the OpModeSelection that was selected in setOpModes()
        // or will return null if the user did not select an OpMode,
        // or an instance of UserDefaultSelection if setOpModes() method is null
        // See the definition of this method for more information.
        if (selectedOpMode == null) {
            telemetry.addRetained("where opmode?");
            return;
        }
        switch (selectedOpMode.toString()) {
            case "Number 1 OpMode":
                // Do something
                break;
            case "Number 2 OpMode":
                // Do something else
                break;
            case "Number 3 OpMode":
                // Do something else again
                break;
        }

        // onReady is where you should assign all of your tasks, using addTask, addTaskFirst, and addTaskLast
        addTask(new WaitTask(Seconds.of(5)));

        // This method is called when the UserSelection phase is done, if you have an initTask running
        // a better place to put code for that is in the overridable onInitDone()/onStart() methods from BunyipsOpMode.
        // Ensure to read the overhead definitions for AutonomousBunyipsOpMode to make sure you do not override any important code.
        // If unsure, make sure to always have a supercall to methods that you explicitly override.
        addTask(new ExampleTask(Seconds.of(3), lift));

        // See the tasks directory for defining your own tasks.
    }
}
