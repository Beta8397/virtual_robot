package org.murraybridgebunyips.imposter.components;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import org.murraybridgebunyips.bunyipslib.RobotConfig;
import org.murraybridgebunyips.bunyipslib.roadrunner.drive.DriveConstants;
import org.murraybridgebunyips.bunyipslib.roadrunner.drive.MecanumCoefficients;
import org.murraybridgebunyips.bunyipslib.roadrunner.drive.localizers.ThreeWheelTrackingLocalizerCoefficients;
import org.murraybridgebunyips.bunyipslib.roadrunner.util.Deadwheel;

import static org.murraybridgebunyips.bunyipslib.external.units.Units.*;

/**
 * Running under "Mecanum Bot" config
 */
public class ImposterConfig extends RobotConfig {
    public DcMotorEx back_right_motor;
    public DcMotorEx back_left_motor;
    public DcMotorEx front_right_motor;
    public DcMotorEx front_left_motor;
    public IMU imu;

    public Servo back_servo;

    public Deadwheel enc_x;
    public Deadwheel enc_left;
    public Deadwheel enc_right;

    public DriveConstants driveConstants;
    public MecanumCoefficients mecanumCoefficients;
    public ThreeWheelTrackingLocalizerCoefficients localizerCoefficients;

    @Override
    protected void onRuntime() {
        back_right_motor = getHardware("back_right_motor", DcMotorEx.class);
        back_left_motor = getHardware("back_left_motor", DcMotorEx.class);
        front_right_motor = getHardware("front_right_motor", DcMotorEx.class);
        front_left_motor = getHardware("front_left_motor", DcMotorEx.class);

        back_servo = getHardware("back_servo", Servo.class);

        enc_x = getHardware("enc_x", Deadwheel.class);
        enc_left = getHardware("enc_left", Deadwheel.class);
        enc_right = getHardware("enc_right", Deadwheel.class);

        enc_left.setDirection(Deadwheel.Direction.REVERSE);

        back_left_motor.setDirection(DcMotorEx.Direction.REVERSE);
        front_left_motor.setDirection(DcMotorEx.Direction.REVERSE);

        // https://github.com/Murray-Bridge-Bunyips/Virtual_BunyipsFTC/blob/master/Road-Runner-Quickstart-Instructions.pdf
        driveConstants = new DriveConstants.Builder()
                .setTicksPerRev(1120)
                .setMaxRPM(160)
                .setRunUsingEncoder(true)
                .setTrackWidth(Inches.of(17.91))
                .setMaxVel(InchesPerSecond.of(21))
                .setMaxAccel(InchesPerSecond.per(Second).of(21))
                .setMaxAngVel(DegreesPerSecond.of(170))
                .setMaxAngAccel(DegreesPerSecond.per(Second).of(170))
                .setKV(1.1)
                .setKA(0.002)
                .build();

        mecanumCoefficients = new MecanumCoefficients.Builder()
                .setTranslationalPID(new PIDCoefficients(2, 0, 0))
                .setHeadingPID(new PIDCoefficients(2.33, 0, 0))
                .build();

        localizerCoefficients = new ThreeWheelTrackingLocalizerCoefficients.Builder()
                .setTicksPerRev(2000)
                .setWheelRadius(Inches.of(2.0))
                .setGearRatio(1)
                .setLateralDistance(Inches.of(12))
                .setForwardOffset(Inches.zero())
                .build();

        imu = getHardware("imu", IMU.class, (d) -> d.initialize(
                new IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                                RevHubOrientationOnRobot.UsbFacingDirection.LEFT
                        )
                )
        ));
    }
}
