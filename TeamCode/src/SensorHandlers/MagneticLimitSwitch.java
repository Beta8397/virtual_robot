package SensorHandlers;

import com.qualcomm.robotcore.hardware.DigitalChannel;

public class MagneticLimitSwitch implements Sensor {
    int id;
    String name;
    DigitalChannel sensor;

    public MagneticLimitSwitch(DigitalChannel s) {
        sensor = s;
        sensor.setMode(DigitalChannel.Mode.INPUT);
    }

    public MagneticLimitSwitch(DigitalChannel s, String name) {
        sensor = s;
        sensor.setMode(DigitalChannel.Mode.INPUT);
        this.name = name;
    }

    public MagneticLimitSwitch(DigitalChannel s, int id) {
        sensor = s;
        sensor.setMode(DigitalChannel.Mode.INPUT);
        this.id = id;
    }

    public boolean isActivated() {
        return !sensor.getState();
    }

    @Override
    public void setId(int id) {
        this.id = id;
    }

    @Override
    public void setName(String n) {
        name = n;
    }

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
