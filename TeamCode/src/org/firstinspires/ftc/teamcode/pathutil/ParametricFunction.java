package org.firstinspires.ftc.teamcode.pathutil;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;

public interface ParametricFunction {
    float p(float s);
    float d1(float s);
    float d2(float s);
}
