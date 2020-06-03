package com.qualcomm.robotcore.hardware;

/**
 * For internal use only. Enumeration of the available dc motor types. Currently limited to Neverest, but
 * would be easy to add more.
 *
 * Properties of a motor type are:
 *      TICKS_PER_ROTATION = encoder ticks per complete output shaft rotation.
 *      MAX_TICKS_PER_SECOND = maximum encoder ticks per second at full speed in RUN_WITH_ENCODER mode.
 *      REVERSED = true if positive power causes shaft to turn counter-clockwise when viewed from free end of shaft,
 *              otherwise false.
 */
public enum MotorType {
    Neverest40(1120, 2500, false),
    Neverest20(560, 2500, false),
    NeverestOrbital20(560, 2500, true),
    Neverest60(1680, 2500, false);

    MotorType(double ticksPerRotation, double maxTicksPerSecond, boolean reversed ){
        TICKS_PER_ROTATION = ticksPerRotation;
        MAX_TICKS_PER_SECOND = maxTicksPerSecond;
        REVERSED = reversed;
    }

    public final double TICKS_PER_ROTATION;
    public final double MAX_TICKS_PER_SECOND;
    public final boolean REVERSED;
}
