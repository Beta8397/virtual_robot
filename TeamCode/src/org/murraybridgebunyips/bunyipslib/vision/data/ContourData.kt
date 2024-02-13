package org.murraybridgebunyips.bunyipslib.vision.data

import org.murraybridgebunyips.bunyipslib.vision.Vision
import org.opencv.core.Rect

data class ContourData(
    val boundingRect: Rect,
) : VisionData() {
    val area: Double = boundingRect.area()
    val aspectRatio: Double = boundingRect.width.toDouble() / boundingRect.height.toDouble()
    val centerX: Double = boundingRect.x + boundingRect.width / 2.0
    val centerY: Double = boundingRect.y + boundingRect.height / 2.0
    val yaw: Double = (centerX - Vision.CAMERA_WIDTH / 2.0) / Vision.CAMERA_WIDTH
    val pitch: Double = (centerY - Vision.CAMERA_HEIGHT / 2.0) / Vision.CAMERA_HEIGHT

    companion object {
        @JvmStatic
        fun getLargest(contours: List<ContourData>): ContourData? {
            return contours.maxByOrNull { it.area }
        }
    }
}
