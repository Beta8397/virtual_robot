package org.murraybridgebunyips.bunyipslib.example.examplerobot.autonomous;

import androidx.annotation.Nullable;

import org.murraybridgebunyips.bunyipslib.AutonomousBunyipsOpMode;
import org.murraybridgebunyips.bunyipslib.OpModeSelection;
import org.murraybridgebunyips.bunyipslib.StartingPositions;
import org.murraybridgebunyips.bunyipslib.drive.CartesianMecanumDrive;
import org.murraybridgebunyips.bunyipslib.example.examplerobot.components.ExampleConfig;
import org.murraybridgebunyips.bunyipslib.example.examplerobot.components.ExampleLift;
import org.murraybridgebunyips.bunyipslib.example.examplerobot.tasks.ExampleTask;
import org.murraybridgebunyips.bunyipslib.tasks.WaitTask;
import org.murraybridgebunyips.bunyipslib.tasks.bases.RobotTask;

import java.util.List;

/**
 * Example autonomous OpMode for a robot with a lift and a mecanum drive.
 */
public class ExampleAutonomous extends AutonomousBunyipsOpMode {
    // This class extends BunyipsOpMode, which is the base class for all OpModes in BunyipsLib
    // If you need to use RoadRunner, it is recommended to use RoadRunnerAutonomousBunyipsOpMode,
    // which is the variant of this class that allows for RoadRunner methods to be used with special
    // methods such as addNewTrajectory()

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

    // You will define your components as normal here, and initialise them in onInitialisation()
    // This procedure is exactly the same as a normal BunyipsOpMode
    private final ExampleConfig config = new ExampleConfig();
    // All components and tasks must be instantiated during runtime, and not in the constructor or member fields.
    private CartesianMecanumDrive drive;
    private ExampleLift lift;

    @Override
    protected void onInitialisation() {
        config.init();
        drive = new CartesianMecanumDrive(config.leftFrontMotor, config.rightFrontMotor, config.leftBackMotor, config.rightBackMotor);
        lift = new ExampleLift(config.liftMotor);
    }

    // setOpModes is used to define dynamic OpMode selection, using an asynchronous UserSelection
    // This is not required, and you can return null if you do not need to use this feature
    // However, if you do need to use this feature, you must return a list of OpModeSelections
    // which will be automatically handled to show up as gamepad inputs when you initialise the OpMode
    // When one of these inputs is pressed, the OpMode will return the corresponding OpModeSelection
    // to the onQueueReady() method, which you can use to determine which OpMode to run.
    @Override
    protected List<OpModeSelection> setOpModes() {
        // A utility from StartingPositions allows the most commonly used setOpModes config,
        // which is STARTING_BLUE_LEFT STARTING_BLUE_RIGHT STARTING_RED_LEFT and STARTING_RED_RIGHT
        return StartingPositions.use();

        // Go immediately to onQueueReady() and ignore this phase
//        return null;

        // Custom implementation
//        return new ArrayList<OpModeSelection>() {{
//            // This is also compatible with Enums and any other object, see OpModeSelection definition.
//            add(new OpModeSelection("Number 1 OpMode"));
//            add(new OpModeSelection("Number 2 OpMode"));
//            add(new OpModeSelection("Number 3 OpMode"));
//        }};
    }

    @Override
    protected RobotTask setInitTask() {
        return null;
    }

    @Override
    protected void onQueueReady(@Nullable OpModeSelection selectedOpMode) {
        // onQueueReady will return the OpModeSelection that was selected in setOpModes()
        // or will return null if the user did not select an OpMode,
        // or an instance of DefaultOpMode if setOpModes() method is null
        // See the definition of this method for more information.
        if (selectedOpMode == null) {
            addRetainedTelemetry("where opmode?");
            return;
        }
        if (selectedOpMode.toString().equals("Number 1 OpMode")) {
            // Do something
        } else if (selectedOpMode.toString().equals("Number 2 OpMode")) {
            // Do something else
        } else if (selectedOpMode.toString().equals("Number 3 OpMode")) {
            // Do something else again
        }

        // onQueueReady is where you should assign all of your tasks, using addTask, addTaskFirst, and addTaskLast
        addTask(new WaitTask(5));

        // This method is called when the UserSelection phase is done, if you have an initTask running
        // a better place to put code for that is in the overridable onInitDone()/onStart() methods from BunyipsOpMode.
        // Ensure to read the overhead definitions for AutonomousBunyipsOpMode to make sure you do not override any important code.
        // If unsure, make sure to always have a supercall to methods that you explicitly override.
        addTask(new ExampleTask(3, lift));

        // See the tasks directory for defining your own tasks.
    }
}
