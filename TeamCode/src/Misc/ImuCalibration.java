package Misc;

import com.qualcomm.hardware.bosch.BNO055IMU;

public class ImuCalibration {

    public static BNO055IMU.CalibrationData getCalibrationData() {
        return new BNO055IMU.CalibrationData();
    }

    public static void logImuStatus(BNO055IMU imu) {
//        Log.d("IMU Status", imu.getSystemStatus().toString());
//        Log.d("IMU Calibration", imu.getCalibrationStatus().toString());
    }
}

