package org.firstinspires.ftc.teamcode.jerry.debug

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.common.BunyipsOpMode
import org.firstinspires.ftc.teamcode.common.OpenCVCam
import org.firstinspires.ftc.teamcode.common.RobotConfig
import org.firstinspires.ftc.teamcode.common.tasks.GetSignalTask
import org.firstinspires.ftc.teamcode.jerry.components.JerryConfig

/**
 * Debug OpMode to continually output what AprilTag position the robot is currently seeing.
 */
@TeleOp(name = "JERRY: PowerPlay Signal Debug", group = "JERRY")
class JerrySignalAnalyse : BunyipsOpMode() {
    private var config = JerryConfig()
    private var cam: OpenCVCam? = null
    private var task: GetSignalTask? = null

    override fun onInit() {
        config = RobotConfig.newConfig(this, config, hardwareMap) as JerryConfig
        cam = OpenCVCam(this, config.webcam, config.monitorID)
        task = cam?.let { GetSignalTask(this, it) }
    }

    override fun activeLoop() {
        task?.run()
        addTelemetry("Currently seeing position: ${task?.position ?: "NONE"}")
    }
}