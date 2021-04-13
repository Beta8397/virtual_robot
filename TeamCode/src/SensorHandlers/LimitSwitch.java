package SensorHandlers;

import com.qualcomm.robotcore.hardware.TouchSensor;

public class LimitSwitch implements Sensor {
    TouchSensor sensor;
    int id;
    String name;

    public LimitSwitch(TouchSensor ts, String name) {
        this.name = name;
        sensor = ts;
    }

    public LimitSwitch(TouchSensor ts, int id) {
        this.id = id;
        sensor = ts;
    }

    public LimitSwitch(TouchSensor ts, int id, String name) {
        this.name = name;
        this.id = id;
        sensor = ts;
    }

    public void setSensor(TouchSensor ts) { sensor = ts; }
    public void setId(int id) { this.id = id; }
    public void setName(String n) { name = n; }

    public boolean isPressed() { return sensor.isPressed(); }
    public double getValue() { return sensor.getValue(); }

    @Override
    public Type getType() {
        return Type.LIMIT_SWITCH;
    }

    @Override
    public int getId() {
        return id;
    }

    @Override
    public String getName() {
        return name;
    }

    @Override
    public void kill() {}
}