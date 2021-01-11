package system.robot.localizer;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.TwoTrackingWheelLocalizer;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;
import system.robot.roadrunner_util.AxesSigns;
import system.robot.roadrunner_util.CoordinateMode;
import system.robot.roadrunner_util.Encoder;
import system.robot.Robot;

import java.util.*;

import static java.lang.Math.PI;
/*
 * Sample tracking wheel localizer implementation assuming the standard configuration:
 *
 *    ^
 *    |
 *    | ( y direction)
 *    |
 *    v
 *    <----( x direction )---->
 *        (forward)
 *    /--------------\
 *    |     ____     |
 *    |     ----     |    <- Perpendicular Wheel
 *    |           || |
 *    |           || |    <- Parallel Wheel
 *    |              |
 *    |              |
 *    \--------------/
 *
 */

/**
 * Taken from https://github.com/NoahBres/road-runner-quickstart/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/drive/TwoWheelTrackingLocalizer.java
 */
public class TwoWheelLocalizer extends TwoTrackingWheelLocalizer {
    private final String PARALLEL_WHEEL, PERPENDICULAR_WHEEL;

    private final BNO055IMU imu;
    private final TrackingWheelConfig trackingWheelConfig;

    private final Map<String, Encoder> encoders = new HashMap<>();

    private double PARALLEL_WEIGHT = 1, PERPENDICULAR_WEIGHT = 1;

    public TwoWheelLocalizer(Robot robot, String imu, BNO055IMU.Parameters imuParameters, String parallelWheel, Pose2d parallelWheelPose, String perpendicularWheel, Pose2d perpendicularWheelPose, TrackingWheelConfig trackingWheelConfig) {
        super(Arrays.asList(parallelWheelPose, perpendicularWheelPose));

        setPoseEstimate(new Pose2d(0,0,0));

        PARALLEL_WHEEL = parallelWheel;
        PERPENDICULAR_WHEEL = perpendicularWheel;

        this.imu = robot.hardwareMap.get(BNO055IMU.class, imu);
        encoders.put(parallelWheel, new Encoder(robot.hardwareMap.get(DcMotorEx.class, parallelWheel)));
        encoders.put(perpendicularWheel, new Encoder(robot.hardwareMap.get(DcMotorEx.class, perpendicularWheel)));

        this.imu.initialize(imuParameters);

        this.trackingWheelConfig = trackingWheelConfig;
    }

    public TwoWheelLocalizer(Robot robot, String imu, String parallelWheel, Pose2d parallelWheelPose, String perpendicularWheel, Pose2d perpendicularWheelPose, TrackingWheelConfig trackingWheelConfig) {
        this(robot, imu, new BNO055IMU.Parameters(), parallelWheel, parallelWheelPose, perpendicularWheel, perpendicularWheelPose, trackingWheelConfig);
    }

    public TwoWheelLocalizer setWeights(double parallelWeight, double perpendicularWeight) {
        PARALLEL_WEIGHT = parallelWeight;
        PERPENDICULAR_WEIGHT = perpendicularWeight;
        return this;
    }

    public TwoWheelLocalizer reverseEncoder(String encoder) {
        Encoder enc = encoders.get(encoder);
        enc.setDirection(enc.getDirection() == Encoder.Direction.FORWARD ? Encoder.Direction.REVERSE : Encoder.Direction.FORWARD);
        return this;
    }

    public TwoWheelLocalizer setEncoderDirection(String encoder, Encoder.Direction encoderDirection) {
        encoders.get(encoder).setDirection(encoderDirection);
        return this;
    }

    public TwoWheelLocalizer remapIMUAxes(AxesOrder axesOrder, AxesSigns axesSigns) {
        //TODO BNO055IMUUtil.remapAxes(imu, axesOrder, axesSigns);
        return this;
    }

    @Override
    public double getHeading() {
        return imu.getAngularOrientation().firstAngle;
    }

    @NotNull
    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(
                trackingWheelConfig.encoderTicksToInches(encoders.get(PARALLEL_WHEEL).getCurrentPosition()) * PARALLEL_WEIGHT,
                trackingWheelConfig.encoderTicksToInches(encoders.get(PERPENDICULAR_WHEEL).getCurrentPosition()) * PERPENDICULAR_WEIGHT
        );
    }

    @Nullable
    @Override
    public List<Double> getWheelVelocities() {
        return Arrays.asList(
                trackingWheelConfig.encoderTicksToInches(encoders.get(PARALLEL_WHEEL).getCorrectedVelocity()) * PARALLEL_WEIGHT,
                trackingWheelConfig.encoderTicksToInches(encoders.get(PERPENDICULAR_WHEEL).getCorrectedVelocity()) * PERPENDICULAR_WEIGHT
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
