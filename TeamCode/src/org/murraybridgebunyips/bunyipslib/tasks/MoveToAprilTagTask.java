package org.murraybridgebunyips.bunyipslib.tasks;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import org.murraybridgebunyips.bunyipslib.BunyipsSubsystem;
import org.murraybridgebunyips.bunyipslib.Controller;
import org.murraybridgebunyips.bunyipslib.EmergencyStop;
import org.murraybridgebunyips.bunyipslib.Inches;
import org.murraybridgebunyips.bunyipslib.roadrunner.drive.RoadRunnerDrive;
import org.murraybridgebunyips.bunyipslib.tasks.bases.ForeverTask;
import org.murraybridgebunyips.bunyipslib.vision.data.AprilTagData;
import org.murraybridgebunyips.bunyipslib.vision.processors.AprilTag;

import java.util.List;
import java.util.Optional;

/**
 * Task to move to and align to an AprilTag.
 *
 * @param <T> the drivetrain to use (must implement RoadRunnerDrive for X pose forward info/FCD)
 * @author Lucas Bubner, 2024
 */
@Config
public class MoveToAprilTagTask<T extends BunyipsSubsystem> extends ForeverTask {
    public static double DESIRED_DISTANCE = Inches.toM(18.0); // m
    public static double SPEED_GAIN = 0.03;
    public static double STRAFE_GAIN = 0.015;
    public static double TURN_GAIN = 0.01;
    public static double MAX_AUTO_SPEED = 0.5;
    public static double MAX_AUTO_STRAFE = 0.5;
    public static double MAX_AUTO_TURN = 0.3;
    public static int TARGET_TAG = -1;

    private final RoadRunnerDrive drive;
    private final AprilTag aprilTag;
    private final Gamepad gamepad;

    public MoveToAprilTagTask(Gamepad gamepad, T drive, AprilTag aprilTag, double desiredDistanceM, double speedGain, double strafeGain, double turnGain, double maxAutoSpeed, double maxAutoStrafe, double maxAutoTurn, int targetTag) {
        super(drive, false);
        if (!(drive instanceof RoadRunnerDrive))
            throw new EmergencyStop("MoveToPixelTask must be used with a drivetrain with X forward Pose/IMU info");
        this.drive = (RoadRunnerDrive) drive;
        this.aprilTag = aprilTag;
        this.gamepad = gamepad;
        DESIRED_DISTANCE = desiredDistanceM;
        SPEED_GAIN = speedGain;
        STRAFE_GAIN = strafeGain;
        TURN_GAIN = turnGain;
        MAX_AUTO_SPEED = maxAutoSpeed;
        MAX_AUTO_STRAFE = maxAutoStrafe;
        MAX_AUTO_TURN = maxAutoTurn;
        TARGET_TAG = targetTag;
    }

    public MoveToAprilTagTask(Gamepad gamepad, T drive, AprilTag aprilTag, int targetTag) {
        this(gamepad, drive, aprilTag, DESIRED_DISTANCE, SPEED_GAIN, STRAFE_GAIN, TURN_GAIN, MAX_AUTO_SPEED, MAX_AUTO_STRAFE, MAX_AUTO_TURN, targetTag);
    }

    @Override
    public void init() {
        if (!aprilTag.isAttached())
            throw new RuntimeException("Vision processor was initialised without being attached to the vision system");
    }

    @Override
    public void periodic() {
        Pose2d pose = Controller.makeRobotPose(gamepad.left_stick_x, gamepad.left_stick_y, gamepad.right_stick_x);

        List<AprilTagData> data = aprilTag.getData();

        Optional<AprilTagData> target = data.stream().filter(p -> TARGET_TAG == -1 || p.getId() == TARGET_TAG).findFirst();
        if (!target.isPresent() || target.get().getRange() == null || target.get().getBearing() == null || target.get().getYaw() == null) {
            drive.setWeightedDrivePower(pose);
            return;
        }

        double rangeError = (target.get().getRange() - Inches.fromM(DESIRED_DISTANCE)) * SPEED_GAIN;
        double yawError = -target.get().getYaw() * STRAFE_GAIN;
        double headingError = target.get().getBearing() * TURN_GAIN;

        drive.setWeightedDrivePower(
                new Pose2d(
                        Range.clip(rangeError, -MAX_AUTO_SPEED, MAX_AUTO_SPEED),
                        Range.clip(yawError, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE),
                        Range.clip(headingError, -MAX_AUTO_TURN, MAX_AUTO_TURN)
                )
        );
    }

    @Override
    public void onFinish() {
//        drive.setSpeedUsingController(0, 0, 0);
    }
}
