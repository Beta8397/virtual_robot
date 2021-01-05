package MotorControllers;

import Misc.Log;

/**
 * Created by Jordan on 8/5/2017.
 */

/*
    A general PID controller class for anything that will use PID
 */
public class PIDController {
    private volatile double Kp = 0;
    private volatile double Ki = 0;
    private volatile double Kd = 0;
    private volatile double setPoint = 0;
    private double error = 0;
    private double P = 0;
    private double I = 0;
    private double D = 0;
    private double tempI = 0;
    private double prevError = 0;
    private long timeAtLastCalculation = 0;
    private double I_CAP = 0;

    public PIDController(double kp, double ki, double kd) {
        Kp = kp;
        Ki = ki;
        Kd = kd;
    }

    public void setP(double p) {Kp = p;}
    public void setI(double i) {Ki = i;}
    public void setD(double d) {Kd = d;}
    public double getP() {return Kp;}
    public double getI() {return Ki;}
    public double getD() {return Kd;}
    public double getI_Max() {return I_CAP;}
    public void setIMax(double cap) {I_CAP = cap;}

    public double getSp(){
        return setPoint;
    }
    public void setSp(double sp) {
        setPoint = sp;
        //reset(); //fix this later, as velocities are non constant prevent motors from acting sporadic 
    }
//  "TURN_Kp":".003",
//          "TURN_Ki":"0.00",
//          "TURN_Ki_MAX":".05",
//          "TURN_Kd":"0.0",

    public double calculatePID(double pointValue) {
        double deltaTime = (System.currentTimeMillis() - timeAtLastCalculation) / 1000.0;
        error = setPoint - pointValue;
        Log.d("PID Error","" + error);
        P = Kp * error;
        Log.d("P","" + P);
        tempI = Ki * error * deltaTime;
        I += tempI;
        if(Ki == 0) I = 0;
        if(I_CAP != 0) {
            if (I > 0 && I > I_CAP) I = I_CAP;
            else if (I < 0 && I < -I_CAP) I = -I_CAP;
        }
        Log.d("I","" + I);
        D = Kd * (error - prevError) / deltaTime;
        prevError = error;
        timeAtLastCalculation = System.currentTimeMillis();
        Log.d("D","" + D);
        if(Double.isNaN(D)) D = 0;
        if(Double.isNaN(I)) I = 0;
        return P + I + D;
    }

    public void reset() {
        P = 0;
        I = 0;
        D = 0;
        timeAtLastCalculation = System.currentTimeMillis();
    }
}
