package org.murraybridgebunyips.bunyipslib.example.examplerobot.autonomous;

import androidx.annotation.Nullable;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.murraybridgebunyips.bunyipslib.Inches;
import org.murraybridgebunyips.bunyipslib.OpModeSelection;
import org.murraybridgebunyips.bunyipslib.RoadRunnerAutonomousBunyipsOpMode;
import org.murraybridgebunyips.bunyipslib.drive.TankDrive;
import org.murraybridgebunyips.bunyipslib.example.examplerobot.components.ExampleConfig;
import org.murraybridgebunyips.bunyipslib.tasks.bases.RobotTask;

import java.util.List;

/**
 * Example RoadRunner autonomous OpMode for a robot with a tank drive.
 */
public class ExampleRoadRunnerAutonomous extends RoadRunnerAutonomousBunyipsOpMode<TankDrive> {
    // This class is an extension of AutonomousBunyipsOpMode that allows integrated RoadRunner methods
    // Read ExampleAutonomous.java for more information on AutonomousBunyipsOpMode

    // Ensure to set the generic type for RoadRunnerAutonomousBunyipsOpMode to your drive class
    // In this instance, we are using a TankDrive, so we set the generic type to TankDrive
    // Drive classes must extend RoadRunnerDrive, defined in BunyipsLib

    // Define configurations as normal
    private final ExampleConfig config = new ExampleConfig();

    @Override
    protected void onInitialise() {
        config.init();
        // No further configuration is required here for setting up drive systems, see setDrive()
        // You can also define your drive here if you wish, but using setDrive() is recommended
        // drive = new TankDrive(...);
    }


    // These methods are the same as defined in AutonomousBunyipsOpMode
    @Override
    protected List<OpModeSelection> setOpModes() {
        return null;
    }

    @Override
    protected RobotTask setInitTask() {
        return null;
    }

    @Override
    protected void onQueueReady(@Nullable OpModeSelection selectedOpMode) {
        // You have access to a range of RoadRunner methods here, primary one being addNewTrajectory
//        addNewTrajectory()
//                .lineToLinearHeading(...)
//                .splineTo(...)
//                .splineTo(...)
//                .splineTo(...)
//                .build();
        addNewTrajectory(new Pose2d(0, 0, 0))
                .forward(Inches.fromMM(1234))
                .buildWithPriority();
        addNewTrajectory()
                .buildWithLowPriority();
        // These methods are syntactic sugar for the following:
//        addTask(new RoadRunnerTask<>(0, drive, drive.trajectoryBuilder().lineToLinearHeading(...).splineTo(...).splineTo(...).splineTo(...).build()));
        // where using buildWithPriority() will add the task to the front of the queue (addTaskFirst()), and
        // buildWithLowPriority() will add the task to the back of the queue (addTaskLast())

        // It is recommended to use the syntactic sugar methods and this class with RoadRunner, as they are more readable and less error prone
        // See the definition of this class to see all the methods available to you
    }

    // The major difference in RoadRunnerAutonomousBunyipsOpMode is that you delegate the drive class
    // to RoadRunner, so you don't need to implement the drive class yourself.
    // If you would like to access your drive methods, which is unlikely, you can instead pass an
    // instance of your drive class here or directly define `drive` in onInitialisation.
    // This method mainly exists for the sake of avoiding headaches by forgetting to set the drive
    // instance, leading to runtime errors. This method will be called after onInitialisation.
    @Override
    protected TankDrive setDrive() {
        return new TankDrive(config.driveConstants, config.coefficients, config.imu, config.leftFrontMotor, config.rightFrontMotor, config.leftBackMotor, config.rightBackMotor);
    }
}
