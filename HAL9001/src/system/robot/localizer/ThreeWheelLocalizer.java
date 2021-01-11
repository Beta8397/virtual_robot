package system.robot.localizer;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;
import system.robot.roadrunner_util.CoordinateMode;
import system.robot.roadrunner_util.Encoder;
import system.robot.Robot;

import java.util.*;

import static java.lang.Math.PI;

public class ThreeWheelLocalizer extends ThreeTrackingWheelLocalizer {

    private double PERPENDICULAR_WEIGHT = 1, PARALLEL_WEIGHT = 1;

    private final String LEFT_WHEEL, RIGHT_WHEEL, FRONT_WHEEL;
    private final Map<String, Encoder> encoders = new HashMap<>();
    private final TrackingWheelConfig trackingWheelConfig;

    public ThreeWheelLocalizer(Robot robot, String leftWheel, Pose2d leftWheelPose, String rightWheel, Pose2d rightWheelPose, String frontWheel, Pose2d frontWheelPose, TrackingWheelConfig trackingWheelConfig) {
        super(Arrays.asList(leftWheelPose, rightWheelPose, frontWheelPose));

        setPoseEstimate(new Pose2d(0,0,0));

        LEFT_WHEEL = leftWheel;
        RIGHT_WHEEL = rightWheel;
        FRONT_WHEEL = frontWheel;

        encoders.put(leftWheel, new Encoder(robot.hardwareMap.get(DcMotorEx.class, leftWheel)));
        encoders.put(rightWheel, new Encoder(robot.hardwareMap.get(DcMotorEx.class, rightWheel)));
        encoders.put(frontWheel, new Encoder(robot.hardwareMap.get(DcMotorEx.class, frontWheel)));

        this.trackingWheelConfig = trackingWheelConfig;
    }

    public ThreeWheelLocalizer(Robot robot, double lateralDistance, double forwardOffset, String leftWheel, String rightWheel, String frontWheel, TrackingWheelConfig trackingWheelConfig) {
        this(
                robot,
                leftWheel,
                new Pose2d(0, lateralDistance / 2, 0),
                rightWheel,
                new Pose2d(0, -lateralDistance / 2, 0),
                frontWheel,
                new Pose2d(forwardOffset, 0, Math.toRadians(90)),
                trackingWheelConfig
        );
    }

    public ThreeWheelLocalizer setWeights(double parallelWeight, double perpendicularWeight) {
        PARALLEL_WEIGHT = parallelWeight;
        PERPENDICULAR_WEIGHT = perpendicularWeight;
        return this;
    }

    public ThreeWheelLocalizer reverseEncoder(String encoder) {
        Encoder enc = encoders.get(encoder);
        enc.setDirection(enc.getDirection() == Encoder.Direction.FORWARD ? Encoder.Direction.REVERSE : Encoder.Direction.FORWARD);
        return this;
    }

    public ThreeWheelLocalizer setEncoderDirection(String encoder, Encoder.Direction encoderDirection) {
        encoders.get(encoder).setDirection(encoderDirection);
        return this;
    }

    @NotNull
    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(
                trackingWheelConfig.encoderTicksToInches(encoders.get(LEFT_WHEEL).getCurrentPosition()) * PARALLEL_WEIGHT,
                trackingWheelConfig.encoderTicksToInches(encoders.get(RIGHT_WHEEL).getCurrentPosition()) * PARALLEL_WEIGHT,
                trackingWheelConfig.encoderTicksToInches(encoders.get(FRONT_WHEEL).getCurrentPosition()) * PERPENDICULAR_WEIGHT
        );
    }

    @NotNull
    @Override
    public List<Double> getWheelVelocities() {
        return Arrays.asList(
                trackingWheelConfig.encoderTicksToInches(encoders.get(LEFT_WHEEL).getCorrectedVelocity()) * PARALLEL_WEIGHT,
                trackingWheelConfig.encoderTicksToInches(encoders.get(RIGHT_WHEEL).getCorrectedVelocity()) * PARALLEL_WEIGHT,
                trackingWheelConfig.encoderTicksToInches(encoders.get(FRONT_WHEEL).getCorrectedVelocity()) * PERPENDICULAR_WEIGHT
        );
    }

    @NotNull
    @Override
    public Pose2d getPoseEstimate() {
        Pose2d rawPose = super.getPoseEstimate();
        return new Pose2d(rawPose.getX(), rawPose.getY(), rawPose.getHeading()-PI/2);
    }

    @Override
    public void setPoseEstimate(@NotNull Pose2d value) {
        super.setPoseEstimate(new Pose2d(value.getX(), value.getY(), value.getHeading()+PI/2));
    }

    @Nullable
    @Override
    public Pose2d getPoseVelocity() {
        return CoordinateMode.ROADRUNNER.convertTo(CoordinateMode.HAL).apply(super.getPoseVelocity());
    }

    @Override
    public void setPoseVelocity(@Nullable Pose2d pose2d) {
        super.setPoseVelocity(CoordinateMode.HAL.convertTo(CoordinateMode.ROADRUNNER).apply(pose2d));
    }
}
