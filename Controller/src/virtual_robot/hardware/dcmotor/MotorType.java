package virtual_robot.hardware.dcmotor;

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
