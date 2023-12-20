package org.firstinspires.ftc.teamcode.common.tasks

//import android.annotation.SuppressLint
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.common.BunyipsOpMode
import org.firstinspires.ftc.teamcode.common.OpenCVCam
import org.firstinspires.ftc.teamcode.common.vision.AprilTagDetectionPipeline

/**
 * Intermediate task for using the AprilTagDetectionPipeline to detect a Signal position during an initLoop.
 * @author Lucas Bubner, FTC 15215; Nov 2022
 */
class GetSignalTask(opMode: BunyipsOpMode, private val cam: OpenCVCam) : Task(opMode),
    AutoTask {
    private lateinit var at: AprilTagDetectionPipeline
    private val lockTimer = ElapsedTime()
    private var noDetections = 0

    // Decimation thresholds, calibrate as needed
    private val decimationHigh = 3f
    private val decimationLow = 2f
    private val decimationHighMetersThreshold = 1.0f
    private val decimationLowThreshold = 4

    /**
     * Get the saved position of where to park.
     * @return An enum of either LEFT, CENTER, or RIGHT determining where to park
     */
    @Volatile
    var position: ParkingPosition? = null
        private set

    enum class ParkingPosition {
        LEFT, CENTER, RIGHT
    }

    override fun init() {
//        if (cam.mode != CamMode.OPENCV) cam.swapModes()

        // Tag size in metres
        val tagsize = 0.166

        // Lens intrinsics calibrations, units in pixels
        // This is calibrated for the Logitech C920 camera, FTC season 2022-2023
        val fx = 578.272
        val fy = 578.272
        val cx = 402.145
        val cy = 221.506
        at = AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy)
        opMode.log("initialised AprilTagDetectionPipeline")
        cam.setPipeline(at)
    }

    override fun isTaskFinished(): Boolean {
        if (position == null) {
            lockTimer.reset()
        }
        // Ensure the signal remains constant for 3 seconds before locking in
        return lockTimer.seconds() >= 3.0
    }

    override fun onFinish() {
        return
    }

    // Suppression is incompatible with virtual_robot
//    @SuppressLint("DefaultLocale")
    override fun run() {
        // Caution! ParkingPosition will be null if the camera does not pick up anything in it's task runtime.
        // Be sure to check if ParkingPosition is null before setting up your specific tasks, to handle a fallback value.
        var newPosition: ParkingPosition? = null
        val detections = at.detectionsUpdate
        // Check if there are new frames
        if (detections != null) {
            // If there are, check if we see any tags
            if (detections.size > 0) {
                // If we do, set parking position based on this information and end the task
                // Will compare detected tags to the APRILTAG_ID array.
                if (detections[0].pose.z < decimationHighMetersThreshold) at.setDecimation(
                    decimationHigh
                )
                for (detection in detections) {
                    when (detection.id) {
                        17 -> {
                            newPosition = ParkingPosition.LEFT
                        }

                        13 -> {
                            newPosition = ParkingPosition.CENTER
                        }

                        7 -> {
                            newPosition = ParkingPosition.RIGHT
                        }

                        else -> {
                            // Must be seeing a different tag, important we ignore it
                        }
                    }
                }
            } else {
                // Otherwise, count these frames since our last detection
                // and change to low decimation if it's been long enough
                noDetections++
                if (noDetections >= decimationLowThreshold) at.setDecimation(decimationLow)
            }
            if (position != newPosition) {
                lockTimer.reset()
            }
            position = newPosition
        }
    }
}