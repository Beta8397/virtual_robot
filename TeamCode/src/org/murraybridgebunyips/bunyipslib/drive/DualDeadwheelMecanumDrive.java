package org.murraybridgebunyips.bunyipslib.drive;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.murraybridgebunyips.bunyipslib.BunyipsOpMode;
import org.murraybridgebunyips.bunyipslib.roadrunner.drive.DriveConstants;
import org.murraybridgebunyips.bunyipslib.roadrunner.drive.MecanumCoefficients;
import org.murraybridgebunyips.bunyipslib.roadrunner.drive.localizers.TwoWheelTrackingLocalizer;
import org.murraybridgebunyips.bunyipslib.roadrunner.drive.localizers.TwoWheelTrackingLocalizerCoefficients;
import org.murraybridgebunyips.bunyipslib.roadrunner.util.Encoder;

/**
 * RoadRunner Mecanum Drive with two tracking dead wheels for localization
 *
 * @author Lucas Bubner, 2023
 */
public class DualDeadwheelMecanumDrive extends MecanumDrive {
    public DualDeadwheelMecanumDrive(@NonNull BunyipsOpMode opMode, DriveConstants constants, MecanumCoefficients mecanumCoefficients, HardwareMap.DeviceMapping<VoltageSensor> voltageSensor, IMU imu, DcMotorEx frontLeft, DcMotorEx frontRight, DcMotorEx backLeft, DcMotorEx backRight, TwoWheelTrackingLocalizerCoefficients localizerCoefficients, Encoder parallel, Encoder perpendicular) {
        super(opMode, constants, mecanumCoefficients, voltageSensor, imu, frontLeft, frontRight, backLeft, backRight);
        setLocalizer(new TwoWheelTrackingLocalizer(localizerCoefficients, parallel, perpendicular, getInstance()));
    }
}
