package org.murraybridgebunyips.imposter.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.murraybridgebunyips.bunyipslib.Direction;
import org.murraybridgebunyips.bunyipslib.subsystems.DualServos;
import org.murraybridgebunyips.bunyipslib.tasks.WaitTask;
import org.murraybridgebunyips.bunyipslib.tasks.bases.Task;
import org.murraybridgebunyips.bunyipslib.tasks.groups.ParallelTaskGroup;

import java.util.Objects;

import static org.murraybridgebunyips.bunyipslib.external.units.Units.*;
import static org.murraybridgebunyips.bunyipslib.external.units.Units.Degrees;

@Autonomous
public class ImposterTwoTest extends ImposterOneTest {
    /** extension/retraction ticks */
    public static int ARM_DELTA = 2000;
    /** angled spike mark, move forward initially, field tiles */
    public static double ANGLED_INITIAL_FORWARD_DIST_FT = 0.8;
    /** forward spike mark, move forward initially, field tiles */
    public static double M_FORWARD_INITIAL_FORWARD_DIST_FT = 0.7;
    /** forward spike mark, forward centimeters */
    public static double M_FORWARD_DIST_CM = 20;
    /** left spike mark, degrees turn */
    public static double M_LEFT_TURN_DEG = 40;
    /** right spike mark, degrees turn */
    public static double M_RIGHT_TURN_DEG = -40;
    @Override
    protected void onStart() {
        super.onStart();

        Task taskOne = new ParallelTaskGroup(
                new WaitTask(1, Seconds).withName("Wait for Arm"),
                makeTrajectory(startingPosition.getPose())
                        .forward(ANGLED_INITIAL_FORWARD_DIST_FT, FieldTile)
                        .withName("Move Forward to Spike Marks")
                        .buildTask()
        ).withName("Move to Spike Marks");

        Task taskTwo = null;
        taskTwo = makeTrajectory(startingPosition.getPose().plus(unitPose(new Pose2d(ANGLED_INITIAL_FORWARD_DIST_FT, 0, 0), FieldTile, Degrees)))
                .turn(M_RIGHT_TURN_DEG, Degrees)
                .withName("Rotate to Right Mark")
                .buildTask();
//        switch (spikeMark) {
//            case FORWARD:
//                taskTwo = makeTrajectory()
//                        .forward(M_FORWARD_DIST_CM, Centimeters)
//                        .withName("Align to Center Mark")
//                        .buildTask();
//                break;
//            case LEFT:
//                taskTwo = makeTrajectory()
//                        .turn(M_LEFT_TURN_DEG, Degrees)
//                        .withName("Rotate to Left Mark")
//                        .buildTask();
//                break;
//            case RIGHT:
//                break;
//        }

//        Task taskThree = claws.openTask(DualServos.ServoSide.LEFT).withName("Open Left Claw");
        Task taskThree = new WaitTask(2, Seconds).withName("TASKTHREE");
//        Task taskFour = arm.deltaTask(-ARM_DELTA).withName("Retract Arm");
        Task taskFour = new WaitTask(2, Seconds).withName("TASKFOUR");
//        Task taskFive = makeTrajectory()
//
//                .buildTask();

        // Add backwards to queue
//        addTaskFirst(taskFive);
        addTaskFirst(taskFour);
        addTaskFirst(taskThree);
        addTaskFirst(Objects.requireNonNull(taskTwo));
        addTaskFirst(taskOne);
    }

}
