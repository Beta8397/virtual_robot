package org.firstinspires.ftc.teamcode.imposter.components;

import androidx.annotation.NonNull;
import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.teamcode.common.BunyipsOpMode;
import org.firstinspires.ftc.teamcode.common.Mecanum;
import org.firstinspires.ftc.teamcode.common.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.common.roadrunner.drive.MecanumCoefficients;
import org.firstinspires.ftc.teamcode.common.roadrunner.drive.MecanumDrive;

public class ImposterDrive extends Mecanum {
    public ImposterDrive(@NonNull BunyipsOpMode opMode, DriveConstants constants, MecanumCoefficients mecanumCoefficients, HardwareMap.DeviceMapping<VoltageSensor> voltageSensor, IMU imu, DcMotorEx frontLeft, DcMotorEx backLeft, DcMotorEx frontRight, DcMotorEx backRight) {
        super(opMode, constants, mecanumCoefficients, voltageSensor, imu, frontLeft, backLeft, frontRight, backRight);
    }
}
