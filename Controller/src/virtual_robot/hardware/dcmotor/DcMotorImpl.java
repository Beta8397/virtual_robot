package virtual_robot.hardware.dcmotor;

import virtual_robot.hardware.DcMotor;

import java.util.Random;

public class DcMotorImpl implements DcMotor {
    public final MotorType MOTOR_TYPE;
    private final Random random = new Random();
    private RunMode mode = RunMode.RUN_WITHOUT_ENCODER;
    private Direction direction = Direction.FORWARD;
    private double power = 0.0;
    private double actualPosition = 0.0;
    private double randomErrorFrac = 0.0;
    private double systematicErrorFrac = 0.0;

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
            actualPosition = 0.0;
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
        int result = (int)Math.floor(actualPosition);
        return MOTOR_TYPE.REVERSED? -result : result;
    }

    public synchronized double getActualPosition(){ return actualPosition; }

    public synchronized void updatePosition(double milliseconds){
        if (mode == RunMode.RUN_TO_POSITION || mode == RunMode.STOP_AND_RESET_ENCODER) return;
        double positionChange = power * MOTOR_TYPE.MAX_TICKS_PER_SECOND * milliseconds / 1000.0;
        positionChange *= (1.0 + systematicErrorFrac + randomErrorFrac * random.nextGaussian());
        if (direction == Direction.FORWARD && MOTOR_TYPE.REVERSED ||
                direction == Direction.REVERSE && !MOTOR_TYPE.REVERSED) positionChange = -positionChange;
        actualPosition += positionChange;
    }

    public synchronized void setRandomErrorFrac(double rdmErrFrac){
        randomErrorFrac = rdmErrFrac;
    }
    public synchronized void setSystematicErrorFrac(double sysErrFrac) { systematicErrorFrac = sysErrFrac; }

}
