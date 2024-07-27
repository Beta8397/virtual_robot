package org.murraybridgebunyips.imposter.autonomous;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.murraybridgebunyips.bunyipslib.*;
import org.murraybridgebunyips.bunyipslib.drive.MecanumDrive;
import org.murraybridgebunyips.bunyipslib.external.pid.PIDController;
import org.murraybridgebunyips.bunyipslib.roadrunner.drive.RoadRunnerDrive;
import org.murraybridgebunyips.bunyipslib.roadrunner.trajectorysequence.TrajectorySequence;
import org.murraybridgebunyips.bunyipslib.tasks.DriveToPoseTask;
import org.murraybridgebunyips.bunyipslib.tasks.RoadRunnerTask;
import org.murraybridgebunyips.bunyipslib.tasks.WaitTask;
import org.murraybridgebunyips.bunyipslib.tasks.groups.ParallelTaskGroup;
import org.murraybridgebunyips.imposter.components.ImposterConfig;

import static org.murraybridgebunyips.bunyipslib.external.units.Units.*;

@Autonomous
public class ImposterCompositeOneTest extends AutonomousBunyipsOpMode implements RoadRunner {
    /**
     * Multiplicative scale for all RoadRunner distances.
     */
    public static double FIELD_TILE_SCALE = 1.1;
    /**
     * X offset to DriveToPose AprilTag
     */
    public static float APRILTAG_FORWARD_OFFSET = -9.0f;
    /**
     * Y offset to DriveToPose AprilTag
     */
    public static float APRILTAG_SIDE_OFFSET = 7.0f;
    /**
     * Position delta (in ticks) of the arm extension at backboard
     */
    public static int ARM_DELTA = 1600;

    private final ImposterConfig config = new ImposterConfig();
//    protected HoldableActuator arm;
//    protected DualServos claws;
//    protected Direction spikeMark;
    private MecanumDrive drive;
//    private Vision vision;
//    private AprilTag aprilTag;
//    private ColourThreshold teamProp;
//    private GetTriPositionContourTask getTeamProp;
    protected StartingPositions startingPosition;

    @Override
    protected void onInitialise() {
        config.init();
//        drive = new TriDeadwheelMecanumDrive(config.driveConstants, config.mecanumCoefficients, hardwareMap.voltageSensor, config.imu, config.front_left_motor, config.front_right_motor, config.back_left_motor, config.back_right_motor, config.localizerCoefficients, config.enc_left, config.enc_right, config.enc_x);
        drive = new MecanumDrive(config.driveConstants, config.mecanumCoefficients, hardwareMap.voltageSensor, config.imu, config.front_left_motor, config.front_right_motor, config.back_left_motor, config.back_right_motor);
//        arm = new HoldableActuator(config.arm).withMovingPower(0.5);
//        claws = new DualServos(config.leftPixel, config.rightPixel, 1.0, 0.0, 0.0, 1.0);
//        vision = new Vision(config.webcam);

//        aprilTag = new AprilTag();
//        AprilTagPoseEstimator atpe = new AprilTagPoseEstimator(aprilTag, drive);
//        onActiveLoop(atpe::update);

        setOpModes(StartingPositions.use());

//        getTeamProp = new GetTriPositionContourTask();
//        setInitTask(getTeamProp);
    }

    @NonNull
    @Override
    public RoadRunnerDrive getDrive() {
        return drive;
    }

    // Set which direction the robot will strafe at the backdrop. Overridden in the right park variant.
    protected RoadRunnerTask afterPixelDropDriveAction(RoadRunnerTrajectoryTaskBuilder builder) {
        return builder
                .strafeLeft(0.95 * FIELD_TILE_SCALE, FieldTile)
                .buildTask();
    }

    @Override
    protected void onReady(@Nullable Reference<?> selectedOpMode, Controls selectedButton) {
        if (selectedOpMode == null)
            return;

        startingPosition = (StartingPositions) selectedOpMode.require();
        // Facing FORWARD from the starting position as selected
        setPose(startingPosition.getPose());

        // Go to backdrop
        Reference<TrajectorySequence> blueRight = Reference.empty();
        TrajectorySequence redLeft = makeTrajectory()
                .forward(1.8 * FIELD_TILE_SCALE, FieldTiles)
                .strafeRight(2.8 * FIELD_TILE_SCALE, FieldTiles)
                .turn(-Math.PI / 2)
                .strafeRight(1 * FIELD_TILE_SCALE, FieldTile)
                .mirrorToRef(blueRight)
                .build();
        TrajectorySequence redRight = makeTrajectory()
                .lineToLinearHeading(startingPosition.getPose()
                        .plus(RoadRunner.unitPose(new Pose2d(1 * FIELD_TILE_SCALE, 1 * FIELD_TILE_SCALE, -90), FieldTiles, Degrees)))
                .build();
        TrajectorySequence blueLeft = makeTrajectory()
                .lineToLinearHeading(startingPosition.getPose()
                        .plus(RoadRunner.unitPose(new Pose2d(1 * FIELD_TILE_SCALE, -1 * FIELD_TILE_SCALE, 90), FieldTiles, Degrees)))
                .build();

        TrajectorySequence targetSequence = null;
        switch (startingPosition) {
            case STARTING_RED_LEFT:
//                teamProp = new RedTeamProp();
                targetSequence = redLeft;
                break;
            case STARTING_RED_RIGHT:
//                teamProp = new RedTeamProp();
                targetSequence = redRight;
                break;
            case STARTING_BLUE_LEFT:
//                teamProp = new BlueTeamProp();
                targetSequence = blueLeft;
                break;
            case STARTING_BLUE_RIGHT:
//                teamProp = new BlueTeamProp();
                targetSequence = blueRight.require();
                break;
        }
//        vision.init(aprilTag, teamProp);
//        vision.start(teamProp);
//        getTeamProp.setProcessor(teamProp);
        assert targetSequence != null;
        makeTrajectory()
                .runSequence(targetSequence)
                .withName("Navigate to Backdrop")
                .addTask();

        // Place pixels and park to the left of the backdrop
//        addTask(arm.deltaTask(ARM_DELTA).withName("Deploy Arm"));
        addTask(new WaitTask(400, Milliseconds).withName("Deploy Arm"));
//        addTask(claws.openTask(DualServos.ServoSide.BOTH).withName("Drop Pixels"));
        addTask(new WaitTask(200, Milliseconds).withName("Drop Pixels"));
        addTask(new WaitTask(Seconds.of(1)).withName("Wait for Pixels"));
        addTask(new ParallelTaskGroup(
                afterPixelDropDriveAction(makeTrajectory()),
                new WaitTask(Seconds.of(1)).withName("Retract")
        ).withName("Stow and Move to Park"));

        makeTrajectory()
                .forward(0.98 * FIELD_TILE_SCALE, FieldTiles)
                .setVelConstraint(atVelocity(0.1, FieldTiles.per(Second)))
                .withName("Finish Park")
                .addTask();
    }

    @Override
    protected void onStart() {
//        spikeMark = getTeamProp.getPosition();
//        int id = SpikeMarkBackdropId.get(spikeMark, startingPosition);
//        AprilTagMetadata aprilTagDetection = AprilTagGameDatabase.getCenterStageTagLibrary().lookupTag(id);
//        if (aprilTagDetection == null) {
//            telemetry.log("apriltag not found, seeing tag: %", id);
//            return;
//        }
//        VectorF targetPos = aprilTagDetection.fieldPosition;
        VectorF targetPos = new VectorF(60.25f, -41.41f, 4f);  // red right
        // Offset from the tag to the backdrop to not drive directly into the board
        targetPos.add(new VectorF(APRILTAG_FORWARD_OFFSET, 0, 0));

        addTaskAtIndex(1, new DriveToPoseTask(Seconds.of(5), drive,
                new Pose2d(targetPos.get(0), targetPos.get(1), 0),
                new PIDController(0.1, 0, 0),
                new PIDController(0.1, 0, 0),
                new PIDController(4, 0, 0)
        ));
//
//        vision.stop(teamProp);
//        vision.start(aprilTag);
    }
}
