package org.firstinspires.ftc.teamcode.common.vision.data

import org.firstinspires.ftc.teamcode.common.vision.TeamProp

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
     * Colour distance of section 1.
     */
    val section1: Double,
    /**
     * Colour distance of section 2.
     */
    val section2: Double,
    /**
     * Colour distance of section 3.
     */
    val section3: Double,
    /**
     * Maximum colour distance of all three sections.
     */
    val maxDistance: Double
) : VisionData()
