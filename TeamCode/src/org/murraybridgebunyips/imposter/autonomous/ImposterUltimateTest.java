package org.murraybridgebunyips.imposter.autonomous;
import static org.murraybridgebunyips.bunyipslib.external.units.Units.Centimeters;
import static org.murraybridgebunyips.bunyipslib.external.units.Units.Degrees;
import static org.murraybridgebunyips.bunyipslib.external.units.Units.FieldTile;
import static org.murraybridgebunyips.bunyipslib.external.units.Units.FieldTiles;
import static org.murraybridgebunyips.bunyipslib.external.units.Units.Second;
import static org.murraybridgebunyips.bunyipslib.external.units.Units.Seconds;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagMetadata;
import org.murraybridgebunyips.bunyipslib.AutonomousBunyipsOpMode;
import org.murraybridgebunyips.bunyipslib.Controls;
import org.murraybridgebunyips.bunyipslib.Direction;
import org.murraybridgebunyips.bunyipslib.Reference;
import org.murraybridgebunyips.bunyipslib.RoadRunner;
import org.murraybridgebunyips.bunyipslib.StartingPositions;
import org.murraybridgebunyips.bunyipslib.drive.DualDeadwheelMecanumDrive;
import org.murraybridgebunyips.bunyipslib.drive.MecanumDrive;
import org.murraybridgebunyips.bunyipslib.external.pid.PIDController;
import org.murraybridgebunyips.bunyipslib.roadrunner.drive.RoadRunnerDrive;
import org.murraybridgebunyips.bunyipslib.roadrunner.trajectorysequence.TrajectorySequence;
import org.murraybridgebunyips.bunyipslib.subsystems.DualServos;
import org.murraybridgebunyips.bunyipslib.subsystems.HoldableActuator;
import org.murraybridgebunyips.bunyipslib.tasks.DriveToPoseTask;
import org.murraybridgebunyips.bunyipslib.tasks.DynamicTask;
import org.murraybridgebunyips.bunyipslib.tasks.GetTriPositionContourTask;
import org.murraybridgebunyips.bunyipslib.tasks.WaitTask;
import org.murraybridgebunyips.bunyipslib.tasks.groups.ParallelTaskGroup;
import org.murraybridgebunyips.bunyipslib.vision.AprilTagPoseEstimator;
import org.murraybridgebunyips.bunyipslib.vision.Vision;
import org.murraybridgebunyips.bunyipslib.vision.processors.AprilTag;
import org.murraybridgebunyips.bunyipslib.vision.processors.ColourThreshold;
import org.murraybridgebunyips.bunyipslib.vision.processors.centerstage.SpikeMarkBackdropId;
import org.murraybridgebunyips.imposter.components.ImposterConfig;
import org.reflections.vfs.Vfs;

/** virtual test opmode for gladosultimatepreload of bunyipsftc */
@Autonomous
public class ImposterUltimateTest extends AutonomousBunyipsOpMode implements RoadRunner {
    // virtual config, the robot is assumed to be perfect with no scaling required
    public static double FIELD_TILE_SCALE = 1;
    public static Direction fakeSpikeMark = Direction.FORWARD;
    public static Direction parkingDirection = Direction.LEFT;


    /** X offset to DriveToPose AprilTag in inches */
    public static float APRILTAG_FORWARD_OFFSET = 9.0f;
    /** Y offset to DriveToPose AprilTag in inches */
    public static float APRILTAG_SIDE_OFFSET = 0;
    /** Position delta (in ticks) of the arm extension at backboard */
    public static int ARM_DELTA_BACKDROP = 1600;
    /** Whether a heading estimate is also used from AprilTag data. */
    public static boolean USING_HEADING_ESTIMATE = true;
    /** Arm to ground from stow in ticks. */
    public static int ARM_DELTA_GROUND = 2000;
    /** Strafe left distance for left park, field tiles. */
    public static double PARK_LEFT_TILES = 1.1;
    /** Strafe right distance for right park, field tiles. Used in the Right Park override. */
    public static double PARK_RIGHT_TILES = 1.1;
    /** Angled spike mark, move forward initially, field tiles */
    public static double ANGLED_INIT_FWD_TILES = 0.8;
    /** Forward spike mark, move forward initially, field tiles */
    public static double M_FORWARD_INIT_FWD_TILES = 0.7;
    /** Forward spike mark, forward centimeters */
    public static double M_FORWARD_ALIGN_FWD_CM = 20;
    /** Left spike mark, degrees turn */
    public static double M_LEFT_ALIGN_TURN_DEG = 40;
    /** Right spike mark, degrees turn */
    public static double M_RIGHT_ALIGN_TURN_DEG = -40;

    private final ImposterConfig config = new ImposterConfig();
    private MecanumDrive drive;
//    private HoldableActuator arm;
//    private DualServos claws;

//    private Vision vision;
//    private AprilTag aprilTag;
//    private ColourThreshold teamProp;
//    private GetTriPositionContourTask getTeamProp;

    private Direction spikeMark;
    private VectorF backdropPose;
    private StartingPositions startingPosition;

    @Override
    protected void onInitialise() {
        config.init();
//        drive = new DualDeadwheelMecanumDrive(
//                config.driveConstants, config.mecanumCoefficients, hardwareMap.voltageSensor, config.imu,
//                config.frontLeft, config.frontRight, config.backLeft, config.backRight, config.localizerCoefficients,
//                config.parallelDeadwheel, config.perpendicularDeadwheel
//        );
        drive = new MecanumDrive(config.driveConstants, config.mecanumCoefficients, hardwareMap.voltageSensor, config.imu, config.front_left_motor, config.front_right_motor, config.back_left_motor, config.back_right_motor);
//        arm = new HoldableActuator(config.arm).withMovingPower(0.5);
//        claws = new DualServos(config.leftPixel, config.rightPixel, 1.0, 0.0, 0.0, 1.0);
//        vision = new Vision(config.webcam);

//        aprilTag = new AprilTag();
//        AprilTagPoseEstimator atpe = new AprilTagPoseEstimator(aprilTag, drive)
//                .setHeadingEstimate(USING_HEADING_ESTIMATE)
//                .setCameraOffset(config.robotCameraOffset);
        // Will run in the background updating the robot pose depending on AprilTag detection
//        onActiveLoop(atpe);

        setOpModes(StartingPositions.use());

//        getTeamProp = new GetTriPositionContourTask();
//        setInitTask(getTeamProp);
    }

    // Set which direction the robot will strafe at the backdrop. Overridden in the right park variant.
    protected Direction getParkingDirection() {
        return parkingDirection;
    }

    @Override
    protected void onReady(@Nullable Reference<?> selectedOpMode, Controls selectedButton) {
        if (selectedOpMode == null)
            return;

        startingPosition = (StartingPositions) selectedOpMode.require();
        // Facing FORWARD from the starting position as selected
        setPose(startingPosition.getPose());

        // Approach Spike Marks, lazy loaded as this is can only be calculated at runtime
        addTask(new DynamicTask(
                () -> new ParallelTaskGroup(
//                        arm.deltaTask(ARM_DELTA_GROUND).withName("Extend Arm"),
                        new WaitTask(Seconds.of(1)).withName("Extend Arm"),
                        makeTrajectory(startingPosition.getPose())
                                .forward(spikeMark == Direction.FORWARD ? M_FORWARD_INIT_FWD_TILES : ANGLED_INIT_FWD_TILES, FieldTile)
                                .withName("Move Forward to Spike Marks")
                                .buildTask()
                )
        ).withName("Move to Spike Marks"));

        // Align arm to Spike Mark, need to defer this task until runtime as this data is not available until then
        addTask(new DynamicTask(() -> {
            if (spikeMark == Direction.FORWARD) {
                // No rotation required
                return makeTrajectory()
                        .forward(M_FORWARD_ALIGN_FWD_CM, Centimeters)
                        .withName("Push to Spike Mark")
                        .buildTask();
            }
            return makeTrajectory()
                    .turn(spikeMark == Direction.LEFT ? M_LEFT_ALIGN_TURN_DEG : M_RIGHT_ALIGN_TURN_DEG, Degrees)
                    .withName("Rotate to Spike Mark")
                    .buildTask();
        }));

        // Place Purple Pixel (loaded left)
//        addTask(claws.openTask(DualServos.ServoSide.LEFT).withName("Open Left Claw"));
        addTask(new WaitTask(Seconds.of(0.2)).withName("Open Left Claw"));
//        addTask(arm.deltaTask(-ARM_DELTA_GROUND).withName("Retract Arm"));
        addTask(new WaitTask(Seconds.of(1)).withName("Retract Arm"));

        // Go to backdrop from current position
        // We do not know where the robot is in terms of a relative space, so these pathways need to be
        // in a global inferred context, accomplished by deferring construction to runtime
        addTask(new DynamicTask(() -> {
            Reference<TrajectorySequence> blue = Reference.empty();
            RoadRunnerTrajectoryTaskBuilder redBuilder = makeTrajectory().mirrorToRef(blue);

            // Recenter to facing forward to restore a known state
            Pose2d currentPos = drive.getPoseEstimate();
            double mult = startingPosition.isBlue() ? -1 : 1;
            redBuilder.splineTo(
                    new Vector2d(currentPos.getX(), currentPos.getY() * mult),
                    startingPosition.getPose().getHeading() * mult
            );

            // Far side backdrop navigation
            if (startingPosition == StartingPositions.STARTING_RED_LEFT || startingPosition == StartingPositions.STARTING_BLUE_RIGHT) {
                if (spikeMark == Direction.FORWARD) {
                    // The Spike Mark is in front of the robot and we are on the far alliance.
                    // Take the route that goes under the Truss, rotate now as we have time too
                    redBuilder.lineToLinearHeading(new Pose2d(-36, -36, 0));
                    // We are 3 field tiles away from the tile before the backdrop
                    redBuilder.forward(3 * FIELD_TILE_SCALE, FieldTiles);
                } else {
                    // We do not have to worry about crashing into the Spike Mark (left or right), take the long way to ensure no collisions.
                    // This ensures the right partner does not have a barreling robot removing their Spike Mark
                    redBuilder.lineTo(new Vector2d(-36, -11.5));
                    // Follow the original path from the L3 OpMode
                    redBuilder
                            .setScale(FIELD_TILE_SCALE)
                            .strafeRight(3, FieldTiles)
                            .turn(-Math.PI / 2)
                            .strafeRight(1, FieldTile);
                }
            } else {
                // Close side backdrop navigation
                boolean onRedWithMarkInterception = startingPosition == StartingPositions.STARTING_RED_RIGHT && spikeMark == Direction.RIGHT;
                boolean onBlueWithMarkInterception = startingPosition == StartingPositions.STARTING_BLUE_LEFT && spikeMark == Direction.LEFT;
                // We're in a dubious position where moving towards the backdrop will push the Spike Mark.
                // Will need to attach a short circumnavigation by moving backwards and strafing right -- the lineTo
                // will then move to the correct position.
                if (onRedWithMarkInterception || onBlueWithMarkInterception) {
                    redBuilder.back(0.7 * FIELD_TILE_SCALE, FieldTiles);
                    redBuilder.strafeRight(0.5 * FIELD_TILE_SCALE, FieldTiles);
                }
                // Move to the backdrop directly in one motion
                redBuilder.lineToLinearHeading(new Pose2d(49, -36, 0));
            }

            // Build and mirror to blue
            TrajectorySequence red = redBuilder.build();

            // Go!
            return makeTrajectory()
                    .runSequence(startingPosition.isRed() ? red : blue.require())
                    .buildTask();
        }).withName("Navigate to Backdrop"));

        // Backdrop pose will be provided after onStart() and has been deferred until runtime
        addTask(new DynamicTask(
                () -> new DriveToPoseTask(Seconds.of(5), drive,
                        new Pose2d(backdropPose.get(0), backdropPose.get(1), 0),
                        new PIDController(0.1, 0, 0),
                        new PIDController(0.1, 0, 0),
                        new PIDController(4, 0, 0))
        ));

        // Place pixels and park to the left of the backdrop
//        addTask(arm.deltaTask(ARM_DELTA_BACKDROP).withName("Deploy Arm"));
        addTask(new WaitTask(Seconds.of(1)).withName("Deploy Arm"));
//        addTask(claws.openTask(DualServos.ServoSide.RIGHT).withName("Drop Pixel"));
        addTask(new WaitTask(Seconds.of(1.5)).withName("Pixel Drop Action"));

        // Park
        double parkDistance = getParkingDirection() == Direction.LEFT ? PARK_LEFT_TILES : -PARK_RIGHT_TILES;
        addTask(new ParallelTaskGroup(
                makeTrajectory(new Pose2d(48, 36 * (startingPosition.isRed() ? -1 : 1), 0))
                        .strafeLeft(parkDistance * FIELD_TILE_SCALE, FieldTiles)
                        .buildTask(),
                new WaitTask(Seconds.of(0.1)),
                new WaitTask(Seconds.of(1.5))
//                claws.closeTask(DualServos.ServoSide.BOTH),
//                arm.deltaTask(-ARM_DELTA_BACKDROP)
        ).withName("Stow and Move to Park"));

        makeTrajectory()
                // Avoid destroying the field wall
                .setVelConstraint(atVelocity(0.4, FieldTiles.per(Second)))
                .forward(0.6 * FIELD_TILE_SCALE, FieldTiles)
                .withName("Finish Park")
                .addTask();

        // Tasks created, run vision initialisation
//        vision.init(aprilTag, teamProp);
//        vision.start(teamProp);
//        teamProp = startingPosition.isRed() ? new RedTeamProp() : new BlueTeamProp();
//        getTeamProp.setProcessor(teamProp);
    }

    @Override
    protected void onStart() {
        spikeMark = fakeSpikeMark;

        // Capture results when PLAY is pressed
//        spikeMark = getTeamProp.getPosition();
        int id = SpikeMarkBackdropId.get(spikeMark, startingPosition);
        AprilTagMetadata aprilTagDetection = AprilTagGameDatabase.getCenterStageTagLibrary().lookupTag(id);
        if (aprilTagDetection == null) {
            telemetry.log("apriltag not found, seeing tag: %", id);
            return;
        }
        // Supply dynamic constructed information for AprilTag alignment tasks
        backdropPose = aprilTagDetection.fieldPosition;
        // Offset from the tag to the backdrop to not drive directly into the board
        backdropPose.subtract(new VectorF(APRILTAG_FORWARD_OFFSET, APRILTAG_SIDE_OFFSET, 0));

//        // Team prop detections are done, switch over to AprilTag now for the rest of the match
//        vision.stop(teamProp);
//        vision.start(aprilTag);
    }

    @NonNull
    @Override
    public RoadRunnerDrive getDrive() {
        return drive;
    }
}
