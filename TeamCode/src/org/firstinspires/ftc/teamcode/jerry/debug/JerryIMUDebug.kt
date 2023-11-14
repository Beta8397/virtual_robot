package org.firstinspires.ftc.teamcode.jerry.debug

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.common.BunyipsOpMode
import org.firstinspires.ftc.teamcode.common.IMUOp
import org.firstinspires.ftc.teamcode.common.RobotConfig
import org.firstinspires.ftc.teamcode.jerry.components.JerryConfig

/**
 * Debug opmode for IMU readouts.
 */
@TeleOp(name = "JERRY: IMU Debug", group = "JERRY")
class JerryIMUDebug : BunyipsOpMode() {
    private var config = JerryConfig()
    private var imu: IMUOp? = null

    override fun onInit() {
        config = RobotConfig.newConfig(this, config, hardwareMap) as JerryConfig
        imu = IMUOp(this, config.imu!!)
        imu?.startCapture()
    }

    override fun activeLoop() {
//        addTelemetry(
//            config?.imu?.getAngularOrientation(
//                AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES
//            )?.thirdAngle.toString()
//        )
        addTelemetry(imu?.heading.toString())
        imu?.tick()
    }
}