package org.murraybridgebunyips.bunyipslib.drive;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.murraybridgebunyips.bunyipslib.roadrunner.drive.DriveConstants;
import org.murraybridgebunyips.bunyipslib.roadrunner.drive.MecanumCoefficients;
import org.murraybridgebunyips.bunyipslib.roadrunner.drive.localizers.TwoWheelTrackingLocalizer;
import org.murraybridgebunyips.bunyipslib.roadrunner.drive.localizers.TwoWheelTrackingLocalizerCoefficients;
import org.murraybridgebunyips.bunyipslib.roadrunner.util.Deadwheel;

/**
 * RoadRunner Mecanum Drive with two tracking dead wheels for localization
 *
 * @author Lucas Bubner, 2023
 * @see MecanumDrive
 */
public class DualDeadwheelMecanumDrive extends MecanumDrive {
    /**
     * Constructs a new DualDeadwheelMecanumDrive.
     *
     * @param constants             the drive constants
     * @param mecanumCoefficients   the mecanum coefficients
     * @param voltageSensor         the hardwareMap voltage sensor
     * @param imu                   the IMU to use
     * @param frontLeft             the front left motor
     * @param frontRight            the front right motor
     * @param backLeft              the back left motor
     * @param backRight             the back right motor
     * @param localizerCoefficients the deadwheel localizer coefficients
     * @param parallel              the parallel deadwheel encoder
     * @param perpendicular         the perpendicular deadwheel encoder
     */
    public DualDeadwheelMecanumDrive(DriveConstants constants, MecanumCoefficients mecanumCoefficients, HardwareMap.DeviceMapping<VoltageSensor> voltageSensor, IMU imu, DcMotorEx frontLeft, DcMotorEx frontRight, DcMotorEx backLeft, DcMotorEx backRight, TwoWheelTrackingLocalizerCoefficients localizerCoefficients, Deadwheel parallel, Deadwheel perpendicular) {
        super(constants, mecanumCoefficients, voltageSensor, imu, frontLeft, frontRight, backLeft, backRight);
        if (!assertParamsNotNull(localizerCoefficients, parallel, perpendicular)) return;
        setLocalizer(new TwoWheelTrackingLocalizer(localizerCoefficients, parallel, perpendicular, getInstance()));
    }
}
