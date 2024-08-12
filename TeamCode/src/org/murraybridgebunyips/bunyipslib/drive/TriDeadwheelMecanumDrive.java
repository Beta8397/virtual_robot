package org.murraybridgebunyips.bunyipslib.drive;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.murraybridgebunyips.bunyipslib.roadrunner.drive.DriveConstants;
import org.murraybridgebunyips.bunyipslib.roadrunner.drive.MecanumCoefficients;
import org.murraybridgebunyips.bunyipslib.roadrunner.drive.localizers.ThreeWheelLocalizer;
import org.murraybridgebunyips.bunyipslib.roadrunner.util.Deadwheel;

import java.util.ArrayList;
import java.util.List;

/**
 * RoadRunner Mecanum Drive with three tracking dead wheels for localization
 *
 * @author Lucas Bubner, 2023
 * @see MecanumDrive
 */
public class TriDeadwheelMecanumDrive extends MecanumDrive {
    /**
     * Create a new TriDeadwheelMecanumDrive
     *
     * @param constants                The drive constants
     * @param mecanumCoefficients      The mecanum coefficients
     * @param voltageSensor            The voltage sensor
     * @param imu                      The IMU
     * @param frontLeft                The front left motor
     * @param frontRight               The front right motor
     * @param backLeft                 The back left motor
     * @param backRight                The back right motor
     * @param localizerCoefficients    The 3 deadwheel localizer coefficients
     * @param enc_left                 The left y encoder
     * @param enc_right                The right y encoder
     * @param enc_x                    The x encoder
     * @param lastTrackingEncPositions The last tracking encoder positions
     * @param lastTrackingEncVels      The last tracking encoder velocities
     */
    public TriDeadwheelMecanumDrive(DriveConstants constants, MecanumCoefficients mecanumCoefficients, HardwareMap.DeviceMapping<VoltageSensor> voltageSensor, IMU imu, DcMotorEx frontLeft, DcMotorEx frontRight, DcMotorEx backLeft, DcMotorEx backRight, ThreeWheelLocalizer.Coefficients localizerCoefficients, Deadwheel enc_left, Deadwheel enc_right, Deadwheel enc_x, List<Integer> lastTrackingEncPositions, List<Integer> lastTrackingEncVels) {
        super(constants, mecanumCoefficients, voltageSensor, imu, frontLeft, frontRight, backLeft, backRight);
        if (!assertParamsNotNull(localizerCoefficients, enc_left, enc_right, enc_x, lastTrackingEncPositions, lastTrackingEncVels))
            return;
        setLocalizer(new ThreeWheelLocalizer(localizerCoefficients, enc_left, enc_right, enc_x, lastTrackingEncPositions, lastTrackingEncVels));
    }

    /**
     * Create a new TriDeadwheelMecanumDrive
     *
     * @param constants             The drive constants
     * @param mecanumCoefficients   The mecanum coefficients
     * @param voltageSensor         The voltage sensor
     * @param imu                   The IMU
     * @param frontLeft             The front left motor
     * @param frontRight            The front right motor
     * @param backLeft              The back left motor
     * @param backRight             The back right motor
     * @param localizerCoefficients The 3 deadwheel localizer coefficients
     * @param enc_left              The left y encoder
     * @param enc_right             The right y encoder
     * @param enc_x                 The x encoder
     */
    public TriDeadwheelMecanumDrive(DriveConstants constants, MecanumCoefficients mecanumCoefficients, HardwareMap.DeviceMapping<VoltageSensor> voltageSensor, IMU imu, DcMotorEx frontLeft, DcMotorEx frontRight, DcMotorEx backLeft, DcMotorEx backRight, ThreeWheelLocalizer.Coefficients localizerCoefficients, Deadwheel enc_left, Deadwheel enc_right, Deadwheel enc_x) {
        super(constants, mecanumCoefficients, voltageSensor, imu, frontLeft, frontRight, backLeft, backRight);
        if (!assertParamsNotNull(localizerCoefficients, enc_left, enc_right, enc_x)) return;
        setLocalizer(new ThreeWheelLocalizer(localizerCoefficients, enc_left, enc_right, enc_x, new ArrayList<>(), new ArrayList<>()));
        updatePoseFromMemory();
    }

    /**
     * Enable overflow compensation if your encoders exceed 32767 counts / second.
     *
     * @return this
     */
    public TriDeadwheelMecanumDrive enableOverflowCompensation() {
        ThreeWheelLocalizer localizer = (ThreeWheelLocalizer) getLocalizer();
        if (localizer != null)
            localizer.enableOverflowCompensation();
        return this;
    }
}
