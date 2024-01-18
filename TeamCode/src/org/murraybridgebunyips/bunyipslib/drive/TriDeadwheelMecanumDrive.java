package org.murraybridgebunyips.bunyipslib.drive;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.murraybridgebunyips.bunyipslib.BunyipsOpMode;
import org.murraybridgebunyips.bunyipslib.roadrunner.drive.DriveConstants;
import org.murraybridgebunyips.bunyipslib.roadrunner.drive.MecanumCoefficients;
import org.murraybridgebunyips.bunyipslib.roadrunner.drive.localizers.StandardTrackingWheelLocalizer;
import org.murraybridgebunyips.bunyipslib.roadrunner.drive.localizers.StandardTrackingWheelLocalizerCoefficients;
import org.murraybridgebunyips.bunyipslib.roadrunner.util.Encoder;

import java.util.ArrayList;
import java.util.List;

/**
 * RoadRunner Mecanum Drive with three tracking dead wheels for localization
 *
 * @author Lucas Bubner, 2023
 */
public class TriDeadwheelMecanumDrive extends MecanumDrive {
    public TriDeadwheelMecanumDrive(@NonNull BunyipsOpMode opMode, DriveConstants constants, MecanumCoefficients mecanumCoefficients, HardwareMap.DeviceMapping<VoltageSensor> voltageSensor, IMU imu, DcMotorEx frontLeft, DcMotorEx frontRight, DcMotorEx backLeft, DcMotorEx backRight, StandardTrackingWheelLocalizerCoefficients localizerCoefficients, Encoder enc_left, Encoder enc_right, Encoder enc_x, List<Integer> lastTrackingEncPositions, List<Integer> lastTrackingEncVels) {
        super(opMode, constants, mecanumCoefficients, voltageSensor, imu, frontLeft, frontRight, backLeft, backRight);
        setLocalizer(new StandardTrackingWheelLocalizer(localizerCoefficients, enc_left, enc_right, enc_x, lastTrackingEncPositions, lastTrackingEncVels));
    }

    public TriDeadwheelMecanumDrive(@NonNull BunyipsOpMode opMode, DriveConstants constants, MecanumCoefficients mecanumCoefficients, HardwareMap.DeviceMapping<VoltageSensor> voltageSensor, IMU imu, DcMotorEx frontLeft, DcMotorEx frontRight, DcMotorEx backLeft, DcMotorEx backRight, StandardTrackingWheelLocalizerCoefficients localizerCoefficients, Encoder enc_left, Encoder enc_right, Encoder enc_x) {
        super(opMode, constants, mecanumCoefficients, voltageSensor, imu, frontLeft, frontRight, backLeft, backRight);
        setLocalizer(new StandardTrackingWheelLocalizer(localizerCoefficients, enc_left, enc_right, enc_x, new ArrayList<>(), new ArrayList<>()));
    }
}
