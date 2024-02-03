package org.murraybridgebunyips.imposter.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.murraybridgebunyips.bunyipslib.drive.TriDeadwheelMecanumDrive;
import org.murraybridgebunyips.bunyipslib.roadrunner.PathRecorder;
import org.murraybridgebunyips.bunyipslib.roadrunner.drive.RoadRunnerDrive;
import org.murraybridgebunyips.imposter.components.ImposterConfig;

@TeleOp(name = "Recorder", group = "VIRTUAL_BUNYIPSFTC")
public class ImposterRecorder extends PathRecorder {
    private ImposterConfig config = new ImposterConfig();

    @Override
    protected void configureRobot() {
        config.init(this);
    }

    @Override
    protected RoadRunnerDrive setDrive() {
        return new TriDeadwheelMecanumDrive(this, config.driveConstants, config.mecanumCoefficients, hardwareMap.voltageSensor, config.imu, config.front_left_motor, config.front_right_motor, config.back_left_motor, config.back_right_motor, config.localizerCoefficients, config.enc_left, config.enc_right, config.enc_x);
    }

    @Override
    protected Pose2d setStartPose() {
        return new Pose2d(0, 0, 0);
    }

    @Override
    protected Vector2d setDeltaThreshold() {
        return new Vector2d(0.1, 0.1);
    }

    @Override
    protected int setSnapshotDurationMs() {
        return 100;
    }
}
