package org.murraybridgebunyips.bunyipslib;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.murraybridgebunyips.bunyipslib.roadrunner.drive.DriveConstants;
import org.murraybridgebunyips.bunyipslib.roadrunner.drive.MecanumCoefficients;

/**
 * Variant of the MecanumDrive that uses field-centric controls, accounting for robot heading.
 *
 * @author Lucas Bubner, 2023
 */
public class FieldCentricMecanumDrive extends MecanumDrive {
    public FieldCentricMecanumDrive(@NonNull BunyipsOpMode opMode, DriveConstants constants, MecanumCoefficients mecanumCoefficients, HardwareMap.DeviceMapping<VoltageSensor> voltageSensor, IMU imu, DcMotorEx frontLeft, DcMotorEx backLeft, DcMotorEx frontRight, DcMotorEx backRight) {
        super(opMode, constants, mecanumCoefficients, voltageSensor, imu, frontLeft, backLeft, frontRight, backRight);
    }

    @Override
    public void setSpeedUsingController(double x, double y, double r) {
        double heading = getExternalHeading();
        double sin = Math.sin(heading);
        double cos = Math.cos(heading);
        super.setSpeedUsingController(
                x * cos - y * sin,
                x * sin + y * cos,
                r
        );
    }

    @Override
    public void setWeightedDrivePower(Pose2d drivePower) {
        double heading = getExternalHeading();
        double sin = Math.sin(heading);
        double cos = Math.cos(heading);
        super.setWeightedDrivePower(new Pose2d(
                drivePower.getX() * cos - drivePower.getY() * sin,
                drivePower.getX() * sin + drivePower.getY() * cos,
                drivePower.getHeading()
        ));
    }
}
