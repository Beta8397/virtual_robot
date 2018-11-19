package hardware;

import java.util.HashMap;

public interface HardwareMap {
    public final HashMap<String, DCMotor> dcMotor = new HashMap<>();
    public final HashMap<String, ColorSensor> colorSensor = new HashMap<>();
    public final HashMap<String, GyroSensor> gyroSensor = new HashMap<>();
}
