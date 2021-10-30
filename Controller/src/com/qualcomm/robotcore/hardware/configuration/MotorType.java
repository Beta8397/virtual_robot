package com.qualcomm.robotcore.hardware.configuration;

/**
 * For internal use only. Enumeration of the available dc motor types. Currently limited to Neverest, but
 * would be easy to add more.
 *
 * Properties of a motor type are:
 *      TICKS_PER_ROTATION = encoder ticks per complete output shaft rotation.
 *      MAX_TICKS_PER_SECOND = maximum encoder ticks per second at full speed in RUN_WITH_ENCODER mode.
 *      REVERSED = true if positive power causes shaft to turn counter-clockwise when viewed from free end of shaft,
 *              otherwise false.
 *      GEARING = gear ratio.
 *      ACHIEVABLE_MAX_RPM_FRACTION = MAX_TICKS_PER_SECOND as a fraction of the maximum ticks per second that could
 *              be achieved in RUN_WITHOUT_ENCODER mode, under zero-load.
 *
 */
public enum MotorType {
    Neverest40(1120, 2500, false, 40, 0.85),
    Neverest20(560, 2500, false, 20, 0.85),
    NeverestOrbital20(560, 2500, true, 20, 0.85),
    Neverest60(1680, 2500, false, 60, 0.85),
    Gobilda192(537.6,2500,false,19.2,0.85),
    Gobilda137(383.6, 2500, false, 13.7, 0.85),
    RevUltraPlanetaryOneToOne(28, 2800, false, 1, 1.0);

    MotorType(double ticksPerRotation, double maxTicksPerSecond, boolean reversed, double gearing,
              double achievableMaxRPMFraction){
        TICKS_PER_ROTATION = ticksPerRotation;
        MAX_TICKS_PER_SECOND = maxTicksPerSecond;
        REVERSED = reversed;
        GEARING = gearing;
        ACHIEVABLE_MAX_RPM_FRACTION = achievableMaxRPMFraction;
        MAX_RPM = 60.0 * MAX_TICKS_PER_SECOND / (TICKS_PER_ROTATION * ACHIEVABLE_MAX_RPM_FRACTION);
    }

    public final double TICKS_PER_ROTATION;
    public final double MAX_TICKS_PER_SECOND;       // Max Ticks Per Sec in RUN_USING_ENCODER mode
    public final boolean REVERSED;
    public final double GEARING;
    public final double ACHIEVABLE_MAX_RPM_FRACTION;
    public final double MAX_RPM;                    // Max RPM in RUN_WITHOUT_ENCODER mode
}
