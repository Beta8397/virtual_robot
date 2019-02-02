package virtual_robot.hardware;

import virtual_robot.controller.VirtualRobotController;

import java.util.HashMap;

/**
 * HardwareMap provides access to the virtual robot hardware
 */
public interface HardwareMap {
    public final HashMap<String, VirtualRobotController.DCMotorImpl> dcMotor = new HashMap<>();
    public final HashMap<String, VirtualRobotController.ColorSensorImpl> colorSensor = new HashMap<>();
    public final HashMap<String, VirtualRobotController.GyroSensorImpl> gyroSensor = new HashMap<>();
    public final HashMap<String, VirtualRobotController.ServoImpl> servo = new HashMap<>();

    <T> T get( java.lang.Class<? extends T> classOrInterface, String deviceName);
}
