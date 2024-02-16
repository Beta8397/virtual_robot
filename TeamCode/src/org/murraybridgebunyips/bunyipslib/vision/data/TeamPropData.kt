package org.murraybridgebunyips.bunyipslib.vision.data

import org.murraybridgebunyips.bunyipslib.vision.processors.centerstage.TeamProp

/**
 * Utility data structure for Team Prop detections.
 * @author Lucas Bubner, 2023
 */
data class TeamPropData(
    /**
     * Position of the prop in the image.
     */
    val position: TeamProp.Positions,
    /**
     * Colour distance of section 1 (left).
     */
    val section1: Double,
    /**
     * Colour distance of section 2 (right).
     */
    val section2: Double,
    /**
     * Maximum colour distance of both sections.
     */
    val maxDistance: Double
) : VisionData()
