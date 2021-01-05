package SensorHandlers;

public interface Sensor {
    enum Type { LIDAR_SENSOR, NONE, LIMIT_SWITCH, ULTRASONIC_AND_IR_DISTANCE_SENSOR};

    void setId(int id);
    void setName(String n);
    Type getType();
    int getId();
    String getName();
    void kill();
}
