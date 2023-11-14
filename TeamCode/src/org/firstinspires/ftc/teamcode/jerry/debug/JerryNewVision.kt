package org.firstinspires.ftc.teamcode.jerry.debug

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.common.BunyipsOpMode
import org.firstinspires.ftc.teamcode.common.RobotConfig
import org.firstinspires.ftc.teamcode.common.Vision
import org.firstinspires.ftc.teamcode.jerry.components.JerryConfig

/**
 * Test opmode for new vision wrappers
 */
//@Disabled
@TeleOp(name = "JERRY: Test Vision Wrapper", group = "JERRY")
class JerryNewVision : BunyipsOpMode() {
    private var config = JerryConfig()
    private var vision: Vision? = null

    override fun onInit() {
        config = RobotConfig.newConfig(this, config, hardwareMap) as JerryConfig
        vision = Vision(this, config.webcam!!)
//        vision?.init(Vision.Processors.APRILTAG)
//        vision?.start(Vision.Processors.APRILTAG)
    }

    override fun activeLoop() {
//        vision?.tick()
//        addTelemetry("FPS: ${vision?.fps}")
//        addTelemetry("Status: ${vision?.status}")
//        addTelemetry("TFOD: ${vision?.tfodData}")
//        addTelemetry("AprilTag: ${vision?.aprilTagData}")
    }
}