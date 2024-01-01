package org.murraybridgebunyips.bunyipslib.vision.data

/**
 * Utility data structure for TensorFlow Object Detections.
 */
data class TfodData(
    /**
     * Label of the detected object.
     */
    val label: String,
    /**
     * Confidence of the detected object.
     */
    val confidence: Float,
    /**
     * Left coordinate of the rectangle bounding the detected object.
     */
    val left: Float,
    /**
     * Right coordinate of the rectangle bounding the detected object.
     */
    val right: Float,
    /**
     * Top coordinate of the rectangle bounding the detected object.
     */
    val top: Float,
    /**
     * Bottom coordinate of the rectangle bounding the detected object.
     */
    val bottom: Float,
    /**
     * Width of the rectangle bounding the detected object.
     */
    val width: Float,
    /**
     * Height of the rectangle bounding the detected object.
     */
    val height: Float,
    /**
     * Width of the entire image.
     */
    val imageWidth: Int,
    /**
     * Height of the entire image.
     */
    val imageHeight: Int,
    /**
     * Estimation of the horizontal angle to the detected object in degrees.
     */
    val horizontalAngleDeg: Double,
    /**
     * Estimation of the horizontal angle to the detected object in radians.
     */
    val horizontalAngleRad: Double
) : VisionData() {
    /**
     * Calculate the 0-1 range of which the object is horizontally translated in the image.
     */
    fun getHorizontalTranslation(): Float {
        return (left + right) / 2 / imageWidth
    }

    /**
     * Calculate the 0-1 range of which the object is vertically translated in the image.
     */
    fun getVerticalTranslation(): Float {
        return (top + bottom) / 2 / imageHeight
    }
}
