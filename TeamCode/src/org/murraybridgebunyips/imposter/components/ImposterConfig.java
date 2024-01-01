package org.murraybridgebunyips.imposter.components;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import org.murraybridgebunyips.bunyipslib.Inches;
import org.murraybridgebunyips.bunyipslib.RobotConfig;
import org.murraybridgebunyips.bunyipslib.roadrunner.drive.DriveConstants;
import org.murraybridgebunyips.bunyipslib.roadrunner.drive.MecanumCoefficients;
import org.murraybridgebunyips.bunyipslib.roadrunner.drive.localizers.TwoWheelTrackingLocalizerCoefficients;
import org.murraybridgebunyips.bunyipslib.roadrunner.util.Encoder;

/**
 * Running under "Mecanum Bot" config
 */
public class ImposterConfig extends RobotConfig {
    public DcMotorEx back_right_motor;
    public DcMotorEx back_left_motor;
    public DcMotorEx front_right_motor;
    public DcMotorEx front_left_motor;
    public IMU imu;

    public DriveConstants driveConstants;
    public MecanumCoefficients mecanumCoefficients;

    @Override
    protected void configureHardware() {
        back_right_motor = (DcMotorEx) getHardware("back_right_motor", DcMotorEx.class);
        back_left_motor = (DcMotorEx) getHardware("back_left_motor", DcMotorEx.class);
        front_right_motor = (DcMotorEx) getHardware("front_right_motor", DcMotorEx.class);
        front_left_motor = (DcMotorEx) getHardware("front_left_motor", DcMotorEx.class);

        back_left_motor.setDirection(DcMotorEx.Direction.REVERSE);
        front_left_motor.setDirection(DcMotorEx.Direction.REVERSE);

        // https://github.com/Murray-Bridge-Bunyips/Virtual_BunyipsFTC/blob/master/Road-Runner-Quickstart-Instructions.pdf
        driveConstants = new DriveConstants.Builder()
                .setTicksPerRev(1120)
                .setMaxRPM(160)
                .setRunUsingEncoder(true)
                .setTrackWidth(17.91)
                .setMaxVel(21)
                .setMaxAccel(21)
                .setMaxAngVel(Math.toRadians(170))
                .setMaxAngAccel(Math.toRadians(170))
                .setKV(1.1)
                .setKA(0.002)
                .build();

        mecanumCoefficients = new MecanumCoefficients.Builder()
                .setTranslationalPID(new PIDCoefficients(2, 0, 0))
                .setHeadingPID(new PIDCoefficients(1, 0, 0))
                .build();

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
