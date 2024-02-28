package org.murraybridgebunyips.bunyipslib.vision.data

import org.murraybridgebunyips.bunyipslib.vision.Vision
import org.opencv.core.Rect

data class ContourData(
    val boundingRect: Rect,
    val area: Double,
    val areaPercent: Double,
    val aspectRatio: Double,
    val centerX: Double,
    val centerY: Double,
    val yaw: Double,
    val pitch: Double
) : VisionData() {
    constructor(boundingRect: Rect) : this(
        boundingRect,
        boundingRect.area(),
        boundingRect.area() / (Vision.CAMERA_WIDTH * Vision.CAMERA_HEIGHT) * 100.0,
        boundingRect.width.toDouble() / boundingRect.height.toDouble(),
        boundingRect.x + boundingRect.width / 2.0,
        boundingRect.y + boundingRect.height / 2.0,
        (((boundingRect.x + boundingRect.width / 2.0) - Vision.CAMERA_WIDTH / 2.0) / (Vision.CAMERA_WIDTH / 2.0)),
        -(((boundingRect.y + boundingRect.height / 2.0) - Vision.CAMERA_HEIGHT / 2.0) / (Vision.CAMERA_HEIGHT / 2.0))
    )

    companion object {
        @JvmStatic
        fun getLargest(contours: List<ContourData>): ContourData? {
            return contours.maxByOrNull { it.area }
        }
    }
}
