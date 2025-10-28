package org.firstinspires.ftc.teamcode.pathutil;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;

public interface ParametricFunction2D {
    VectorF p(float s);
    VectorF d1(float s);
    VectorF d2(float s);
}
