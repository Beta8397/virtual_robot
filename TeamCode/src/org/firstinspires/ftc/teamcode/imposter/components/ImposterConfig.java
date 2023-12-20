package org.firstinspires.ftc.teamcode.imposter.components;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.teamcode.common.Inches;
import org.firstinspires.ftc.teamcode.common.RobotConfig;
import org.firstinspires.ftc.teamcode.common.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.common.roadrunner.drive.MecanumCoefficients;
import org.firstinspires.ftc.teamcode.common.roadrunner.drive.localizers.TwoWheelTrackingLocalizerCoefficients;
import org.firstinspires.ftc.teamcode.common.roadrunner.util.Encoder;

/**
 * Running under "Mecanum Bot" config
 */
public class ImposterConfig extends RobotConfig {
    public DcMotorEx back_right_motor;
    public DcMotorEx back_left_motor;
    public DcMotorEx front_right_motor;
    public DcMotorEx front_left_motor;
    public DriveConstants driveConstants;
    public TwoWheelTrackingLocalizerCoefficients localizerCoefficients;
    public MecanumCoefficients mecanumCoefficients;
    public Encoder parallelEncoder;

    public Encoder perpendicularEncoder;
    public IMU imu;

    @Override
    protected void configureHardware() {
        back_right_motor = (DcMotorEx) getHardware("back_right_motor", DcMotorEx.class);
        back_left_motor = (DcMotorEx) getHardware("back_left_motor", DcMotorEx.class);
        front_right_motor = (DcMotorEx) getHardware("front_right_motor", DcMotorEx.class);
        front_left_motor = (DcMotorEx) getHardware("front_left_motor", DcMotorEx.class);

        back_left_motor.setDirection(DcMotorEx.Direction.REVERSE);
        front_left_motor.setDirection(DcMotorEx.Direction.REVERSE);

        localizerCoefficients = new TwoWheelTrackingLocalizerCoefficients.Builder()
                .build();

        driveConstants = new DriveConstants.Builder()
                .setTicksPerRev(537.6)
                .setMaxRPM(312.5)
                .setRunUsingEncoder(false)
                .setWheelRadius(1.4763)
                .setGearRatio(1)
                .setTrackWidth(18)
                // ((MAX_RPM / 60) * GEAR_RATIO * WHEEL_RADIUS * 2 * Math.PI) * 0.85
                .setMaxVel(41.065033847087705)
                .setMaxAccel(41.065033847087705)
                .setMaxAngVel(Math.toRadians(130.71406249999998))
                .setMaxAngAccel(Math.toRadians(130.71406249999998))
                .build();

        localizerCoefficients = new TwoWheelTrackingLocalizerCoefficients.Builder()
                .setTicksPerRev(2400)
                .setGearRatio(1)
                .setWheelRadius(Inches.fromMM(50) / 2)
                .setParallelX(0)
                .setParallelY(0)
                .setPerpendicularX(0)
                .setPerpendicularY(0)
                .build();

        mecanumCoefficients = new MecanumCoefficients.Builder()
                .build();

        DcMotorEx pe = (DcMotorEx) getHardware("enc_x", DcMotorEx.class);
        if (pe != null) {
            parallelEncoder = new Encoder(pe);
        }

        DcMotorEx ppe = (DcMotorEx) getHardware("enc_right", DcMotorEx.class);
        if (ppe != null) {
            perpendicularEncoder = new Encoder(ppe);
        }

        imu = (IMU) getHardware("imu", IMU.class);
        // i swear to god if my virtual hardware becomes null
        assert imu != null;
        imu.initialize(
                new IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                                RevHubOrientationOnRobot.UsbFacingDirection.LEFT
                        )
                )
        );
    }
}
