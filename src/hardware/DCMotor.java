package hardware;

public interface DCMotor {

    public enum Direction {FORWARD, REVERSE}

    public enum RunMode {RUN_TO_POSITION, RUN_USING_ENCODER, RUN_WITHOUT_ENCODER, STOP_AND_RESET_ENCODER}

    public void setMode(RunMode mode);
    public RunMode getMode();
    public void setDirection(Direction direction);
    public Direction getDirection();
    public void setPower(double power);
    public double getPower();
    public int getCurrentPosition();

}
