package org.firstinspires.ftc.teamcode.pedroPathing.tuning.tuning_util;

import com.qualcomm.robotcore.hardware.Gamepad;

public class Adjuster {
    public interface Value extends Getter, Setter{}
    public interface Getter{double get();}
    public interface Setter{void set(double v);}
    Value value;
    String name;
    Gamepad gamepad;

    public Adjuster(String name, Getter getter, Setter setter, Gamepad gamepad){
        this.name = name;
        this.value = new Value() {
            @Override
            public double get() { return getter.get(); }

            @Override
            public void set(double v) { setter.set(v); }
        };
        this.gamepad = gamepad;
    }

    public String getName(){return name;}
    public double getValue(){return value.get();}

    public double update(){
        double v = value.get();
        if (gamepad.dpadUpWasPressed()){
            if (Math.abs(v) >= 0.000005) {
                v *= 10.0;
            } else {
                v = 0.00001;
            }
        } else if (gamepad.dpadDownWasPressed()){
            if (Math.abs(v) >= 0.00005) {
                v /= 10.0;
            } else {
                v = 0;
            }
        } else if (gamepad.dpadRightWasPressed()){
            if (Math.abs(v) >= 0.000005) {
                v += Math.pow(10, Math.floor(Math.log10(v)) - 1);
            } else {
                v = 0.00001;
            }
        } else if (gamepad.dpadLeftWasPressed()) {
            if (Math.abs(v) >= 0.000015) {
                v -= Math.pow(10, Math.floor(Math.log10(v)) - 1);
            } else {
                v = 0;
            }
        }
        value.set(v);
        return v;
    }
}
