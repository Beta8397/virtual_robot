package org.firstinspires.ftc.teamcode.jerry.autonomous

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import org.firstinspires.ftc.teamcode.common.BunyipsOpMode
import org.firstinspires.ftc.teamcode.common.NullSafety
import org.firstinspires.ftc.teamcode.common.OpenCVCam
import org.firstinspires.ftc.teamcode.common.RobotConfig
import org.firstinspires.ftc.teamcode.common.tasks.AutoTask
import org.firstinspires.ftc.teamcode.common.tasks.GetSignalTask
import org.firstinspires.ftc.teamcode.jerry.components.JerryConfig
import org.firstinspires.ftc.teamcode.jerry.components.JerryDrive
import org.firstinspires.ftc.teamcode.jerry.tasks.JerryTimeDriveTask
import java.util.*

/**
 * Basic Signal read and park OpMode. Uses camera to read the signal and then drives to the correct square.
 * Uses TimeDrive (which is deprecated), but works.
 */
@Disabled
@Autonomous(
    name = "JERRY: PowerPlay Signal Read & Park w/ TIME DRIVE",
    group = "JERRY",
//    preselectTeleOp = "JERRY: TeleOp"
)
class JerrySignalAutonomousBasic : BunyipsOpMode() {
    private var config = JerryConfig()
    private var cam: OpenCVCam? = null
    private var drive: JerryDrive? = null
    private var tagtask: GetSignalTask? = null
    private val tasks = ArrayDeque<AutoTask>()

    override fun onInit() {
        // Configuration of camera and drive components
        config = RobotConfig.newConfig(this, config, hardwareMap) as JerryConfig
        cam = OpenCVCam(this, config.webcam, config.monitorID)
        if (NullSafety.assertNotNull(config.driveMotors))
            drive = JerryDrive(this, config.bl!!, config.br!!, config.fl!!, config.fr!!)

        // Initialisation of guaranteed task loading completed. We can now dedicate our
        // CPU cycles to the init-loop and find the Signal position.
        tagtask = cam?.let { GetSignalTask(this, it) }
    }

    override fun onInitLoop(): Boolean {
        // Using OpenCV and AprilTags in order to detect the Signal sleeve
        tagtask?.run()
        return tagtask?.isFinished() ?: true
    }

    override fun onInitDone() {
        // Determine our final task based on the parking position from the camera
        // If on center or NONE, do nothing and just stay in the center
        val position = tagtask?.position
        addTelemetry("ParkingPosition set to: $position")

        // Add movement tasks based on the signal position
        if (position == GetSignalTask.ParkingPosition.LEFT) {
            // Drive forward if the position of the signal is LEFT
            tasks.add(JerryTimeDriveTask(this, 1.5, drive, 0.0, 1.0, 0.0))
        } else if (position == GetSignalTask.ParkingPosition.RIGHT) {
            // Drive backward if the position of the signal is RIGHT
            tasks.add(JerryTimeDriveTask(this, 1.5, drive, 0.0, -1.0, 0.0))
        }

        tasks.add(JerryTimeDriveTask(this, 1.5, drive, 1.0, 0.0, 0.0))
    }

    override fun activeLoop() {
        val currentTask = tasks.peekFirst()
        if (currentTask == null) {
            finish()
            return
        }
        currentTask.run()
        if (currentTask.isFinished()) {
            tasks.removeFirst()
        }
        if (tasks.isEmpty()) {
            drive?.stop()
        }
    }
}