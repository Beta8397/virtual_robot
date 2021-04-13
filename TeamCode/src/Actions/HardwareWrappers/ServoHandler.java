package Actions.HardwareWrappers;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by robotics on 11/29/17.
 *
 *
 * NOTE! getDegree() and getPosition() return the WANTED CURRENT DEGREE, NOT THE ACTUAL
 *
 */

/*
    A class to handle setting up servos and simple functions of servos for us
 */

public class ServoHandler {

    private Servo servo;
    private double wantedDegree;
    double degreeRange[] = { 0, 180 };
    final static int RANGE_MAX = 1;
    final static int RANGE_MIN = 0;

    public ServoHandler(Servo s) { servo = s; }

    public ServoHandler(String name, HardwareMap hw) { this(hw.servo.get(name)); }

    public void setServoRanges(double minDeg, double maxDeg) {
        if(maxDeg > minDeg){
            degreeRange[RANGE_MAX] = maxDeg;
            degreeRange[RANGE_MIN] = minDeg;
        }
        else throw new RuntimeException("Servo Handler Creation error: min degree greater than max");
    }

    public double setPosition(double pos) {
        if(pos > degreeRange[RANGE_MAX]/180.0) pos = degreeRange[RANGE_MAX]/180.0;
        if(pos < degreeRange[RANGE_MIN]/180.0) pos = degreeRange[RANGE_MIN]/180.0;
        servo.setPosition(pos);
        wantedDegree = pos*180.0;
        return wantedDegree/180.0;
    }

    public double setDegree(double deg) {
        if(deg > degreeRange[RANGE_MAX]) deg = degreeRange[RANGE_MAX];
        if(deg < degreeRange[RANGE_MIN]) deg = degreeRange[RANGE_MIN];
        wantedDegree = deg;
        servo.setPosition(wantedDegree/180.0);
        return wantedDegree;
    }

    public double incrementDegree(double increment) {
        wantedDegree += increment;
        if(wantedDegree > degreeRange[RANGE_MAX]) wantedDegree = degreeRange[RANGE_MAX];
        if(wantedDegree < degreeRange[RANGE_MIN]) wantedDegree = degreeRange[RANGE_MIN];
        servo.setPosition(wantedDegree/180.0);
        return wantedDegree;
    }

    public double getActualPosition() {return servo.getPosition();}

    public double getPosition(){ return wantedDegree/180.0; }

    public double getActualDegree(){ return getActualPosition()*180.0; }

    public double getDegree(){ return wantedDegree; }

    public void setDirection(Servo.Direction dir){ servo.setDirection(dir); }
}
