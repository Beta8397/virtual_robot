package org.murraybridgebunyips.bunyipslib.vision.data

import org.opencv.core.Rect

data class ContourData(
    val boundingRect: Rect,
) : VisionData() {
    val area: Double = boundingRect.area()
    val aspectRatio: Double = boundingRect.width.toDouble() / boundingRect.height.toDouble()
    val centerX: Double = boundingRect.x + boundingRect.width / 2.0
    val centerY: Double = boundingRect.y + boundingRect.height / 2.0
}
