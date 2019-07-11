package virtual_robot.hardware.dcmotor;

import virtual_robot.controller.VirtualRobotController;
import virtual_robot.hardware.DcMotor;

import java.util.Random;

public class DcMotorImpl implements DcMotor {
    public final MotorType MOTOR_TYPE;
    private final Random random = new Random();
    private RunMode mode = RunMode.RUN_WITHOUT_ENCODER;
    private Direction direction = Direction.FORWARD;

    //power is the requested speed, normalized to the -1 to +1 range
    private double power = 0.0;

    //speed is the actual speed, normalized to the -1 to +1 range
    private double speed = 0.0;

    //actual position of motor
    private double actualPosition = 0.0;

    //position to use as baseline for encoder tick calculation
    private double encoderBasePosition = 0.0;

    private double randomErrorFrac = 0.0;
    private double systematicErrorFrac = 0.0;
    private double inertia;

    public DcMotorImpl(){
        MOTOR_TYPE = MotorType.Neverest40;
    }

    public DcMotorImpl(MotorType motorType){
        MOTOR_TYPE = motorType;
    }

    public synchronized void setMode(RunMode mode){
        this.mode = mode;
        if (mode == RunMode.STOP_AND_RESET_ENCODER){
            power = 0.0;
            encoderBasePosition = actualPosition;
        }
    }

    public synchronized RunMode getMode(){ return mode; }
    public synchronized void setDirection(Direction direction){ this.direction = direction; }
    public synchronized Direction getDirection(){ return direction; }
    public synchronized double getPower(){ return power; }

    public synchronized void setPower(double power){
        this.power = Math.max(-1, Math.min(1, power));
    }

    public synchronized int getCurrentPosition(){
        int result = (int)Math.floor(actualPosition - encoderBasePosition);
        return direction == Direction.FORWARD && MOTOR_TYPE.REVERSED ||
                direction == Direction.REVERSE && !MOTOR_TYPE.REVERSED ? -result : result;
    }

    public synchronized double getActualPosition(){ return actualPosition; }

    //Updates motor speed based on current speed, power, and inertia. Then, uses motor speed to update position.
    //Returns change in actual motor position
    public synchronized double update(double milliseconds){
        if (mode == RunMode.RUN_TO_POSITION || mode == RunMode.STOP_AND_RESET_ENCODER) return 0.0;
        speed = speed + (1.0 - inertia) * (power - speed);
        double positionChange = speed * MOTOR_TYPE.MAX_TICKS_PER_SECOND * milliseconds / 1000.0;
        positionChange *= (1.0 + systematicErrorFrac + randomErrorFrac * random.nextGaussian());
        if (direction == Direction.FORWARD && MOTOR_TYPE.REVERSED ||
                direction == Direction.REVERSE && !MOTOR_TYPE.REVERSED) positionChange = -positionChange;
        actualPosition += positionChange;
        return positionChange;
    }

    public synchronized void setRandomErrorFrac(double rdmErrFrac){
        randomErrorFrac = rdmErrFrac;
    }
    public synchronized void setSystematicErrorFrac(double sysErrFrac) { systematicErrorFrac = sysErrFrac; }
    public synchronized void setInertia(double in){
        if (in < 0) inertia = 0.0;
        else if (in > 0.99) inertia = 0.99;
        else inertia = in;
    }

    //For system programming only: for stopping and resetting motor between op mode runs
    public synchronized void stopAndReset(){
        power = 0.0;
        speed = 0.0;
        actualPosition = 0.0;
        encoderBasePosition = 0.0;
        direction = Direction.FORWARD;
    }

}
