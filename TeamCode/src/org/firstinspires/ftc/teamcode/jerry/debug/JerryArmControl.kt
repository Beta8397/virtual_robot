package org.firstinspires.ftc.teamcode.jerry.debug

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.common.BunyipsOpMode
import org.firstinspires.ftc.teamcode.common.NullSafety
import org.firstinspires.ftc.teamcode.common.RobotConfig
import org.firstinspires.ftc.teamcode.jerry.components.JerryConfig
import org.firstinspires.ftc.teamcode.jerry.components.JerryLift

/**
 * Manual arm control used for calibration purposes, using gamepad2 left stick.
 */
@TeleOp(name = "JERRY: Manual Arm Control", group = "JERRY")
class JerryArmControl : BunyipsOpMode() {
    private var config = JerryConfig()
    private var arm: JerryLift? = null

    override fun onInit() {
        config = RobotConfig.newConfig(this, config, hardwareMap) as JerryConfig
        if (NullSafety.assertNotNull(config.armComponents)) {
            arm = JerryLift(
                this,
                JerryLift.ControlMode.MANUAL,
                config.claw!!,
                config.arm1!!,
                config.arm2!!,
//                config.limit!!
            )
        }

    }

    override fun activeLoop() {
        arm?.delta(gamepad2.left_stick_y.toDouble())
        // Calculates the average position of the lift motors
        addTelemetry("Lift Position: ${(config.arm1?.currentPosition!! + config.arm2?.currentPosition!!) / 2}")
        arm?.update()
    }
}