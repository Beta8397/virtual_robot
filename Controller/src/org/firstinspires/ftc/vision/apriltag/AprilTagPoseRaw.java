package org.firstinspires.ftc.vision.apriltag;

import org.firstinspires.ftc.robotcore.external.matrices.MatrixF;

public class AprilTagPoseRaw
{
    /**
     * X translation
     */
    public final double x;

    /**
     * Y translation
     */
    public final double y;

    /**
     * Z translation
     */
    public final double z;

    /**
     * 3x3 rotation matrix
     */
    public final MatrixF R;

    public AprilTagPoseRaw(double x, double y, double z, MatrixF R)
    {
        this.x = x;
        this.y = y;
        this.z = z;
        this.R = R;
    }
}