package org.firstinspires.ftc.teamcode.jerry.tasks

import org.firstinspires.ftc.teamcode.common.BunyipsOpMode
import org.firstinspires.ftc.teamcode.common.IMUOp
import org.firstinspires.ftc.teamcode.common.tasks.AutoTask
import org.firstinspires.ftc.teamcode.common.tasks.Task
import org.firstinspires.ftc.teamcode.jerry.components.JerryDrive

// This tasks only uses the IMU and time in order to drive, to see the implementation of both deadwheel and IMU
// see the PrecisionDrive task
@Deprecated("Use JerryPrecisionDriveTask instead")
class JerryIMUDriveTask(
    opMode: BunyipsOpMode,
    time: Double,
    private val drive: JerryDrive?,
    private val imu: IMUOp?,
    private val x: Double,
    private val y: Double,
    private val r: Double
) : Task(opMode, time), AutoTask {
    override fun init() {
        imu?.startCapture()
    }

    override fun run() {
        drive?.setSpeedUsingController(x, -y, imu?.getRPrecisionSpeed(r, 3.0) ?: 0.0)
        drive?.update()
        imu?.tick()
    }

    override fun isTaskFinished(): Boolean {
        return false
    }

    override fun onFinish() {
        drive?.stop()
        imu?.resetCapture()
    }
}