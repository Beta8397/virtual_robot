package SensorHandlers;

import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class LIDARSensor implements Sensor {

    DistanceSensor lidarSensor;
    int id;
    String name;
    public static final double GOOD_DIST_TOLERANCE = 10;
    private final double OFFSET = 1;
    private double lastDistance = 0, currentDistance = 0;

    public LIDARSensor(DistanceSensor ls, String name) {
        this.name = name;
        lidarSensor = ls;
        initSensor();
    }

    public LIDARSensor(DistanceSensor ls, int id) {
        this.id = id;
        lidarSensor = ls;
        initSensor();
    }

    public LIDARSensor(DistanceSensor ls, int id, String name) {
        this.id = id;
        this.name = name;
        lidarSensor = ls;
        initSensor();
    }

    private void initSensor() { for (int i = 0; i < 50; i++) getDistance(); }

    public void setSensor(DistanceSensor s) {
        this.lidarSensor = s;
        initSensor();
    }
    public void setId(int id) { this.id = id; }
    public void setName(String n) { this.name = n; }

    public double getDistance(){
        lastDistance = currentDistance;
        currentDistance = lidarSensor.getDistance(DistanceUnit.INCH) * OFFSET;
        return currentDistance;
    }

    public Double getGoodDistance() {
        double distToCheck = getDistance();
        if(Math.abs(distToCheck - lastDistance) < GOOD_DIST_TOLERANCE) return distToCheck;
        return null;
    }

    @Override
    public Type getType() { return Type.LIDAR_SENSOR; }

    @Override
    public int getId() { return id; }

    @Override
    public String getName() { return name; }

    @Override
    public void kill() {}
}
