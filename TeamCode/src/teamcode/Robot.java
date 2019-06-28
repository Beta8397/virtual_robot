package teamcode;

import virtual_robot.hardware.*;
import virtual_robot.hardware.bno055.BNO055IMU;
import virtual_robot.util.navigation.AngleUnit;
import virtual_robot.util.navigation.AxesOrder;
import virtual_robot.util.navigation.AxesReference;
import virtual_robot.util.navigation.Orientation;

public class Robot {
    private BNO055IMU imu;
    private MecanumDrive mecanumDrive = new MecanumDrive();

    void init(HardwareMap hwMap) {
        imu = hwMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters params = new BNO055IMU.Parameters();
        imu.initialize(params);
        mecanumDrive.init(hwMap);
    }

    double getHeadingRadians() {
        Orientation angles;

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        return -angles.firstAngle;   // Not sure why this is negative, could be the simulator

    }

    double degreeFromRadians(double theta) {
        return theta * 360 / (2 * Math.PI);
    }

    void driveFieldRelative(Telemetry telemetry, double x, double y, double rotate) {
        //Convert x, y to theta, r

        double r = Math.sqrt(x * x + y * y);
        double theta = Math.atan2(y, x);

        //Get modified theta, r based off gyro heading
        double heading = getHeadingRadians();

        double modifiedTheta = theta - heading;
        if (modifiedTheta < -Math.PI) {
            modifiedTheta += 2 * Math.PI;
        } else if (modifiedTheta > Math.PI) {
            modifiedTheta -= 2 * Math.PI;
        }

        //Convert theta and r back to a forward and strafe
        double forward = r * Math.cos(modifiedTheta);
        double strafe = r * Math.sin(modifiedTheta);

        mecanumDrive.driveMecanum(forward, strafe, rotate);
    }
}
