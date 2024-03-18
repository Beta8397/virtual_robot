package org.murraybridgebunyips.bunyipslib.vision.data

import org.murraybridgebunyips.bunyipslib.vision.Vision
import org.opencv.core.Rect

/**
 * Data class for storing contour data.
 */
data class ContourData(
    /**
     * The bounding rectangle of the contour.
     */
    val boundingRect: Rect,
    /**
     * The area of the contour.
     */
    val area: Double,
    /**
     * The percentage of the screen the contour takes up.
     */
    val areaPercent: Double,
    /**
     * The aspect ratio of the contour.
     */
    val aspectRatio: Double,
    /**
     * The x coordinate of the center of the contour.
     */
    val centerX: Double,
    /**
     * The y coordinate of the center of the contour.
     */
    val centerY: Double,
    /**
     * The yaw of the contour in degrees.
     */
    val yaw: Double,
    /**
     * The pitch of the contour in degrees.
     */
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
        /**
         * Get the largest contour from a list of contours.
         */
        @JvmStatic
        fun getLargest(contours: List<ContourData>): ContourData? {
            return contours.maxByOrNull { it.area }
        }
    }
}
