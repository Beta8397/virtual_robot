package virtual_robot.hardware;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * HardwareMap provides access to the virtual robot hardware
 */
public class HardwareMap {

    public final DeviceMapping<DCMotor> dcMotor = new DeviceMapping<>(DCMotor.class);
    public final DeviceMapping<ColorSensor> colorSensor = new DeviceMapping<>(ColorSensor.class);
    public final DeviceMapping<GyroSensor> gyroSensor = new DeviceMapping<>(GyroSensor.class);
    public final DeviceMapping<Servo> servo = new DeviceMapping<>(Servo.class);

    private Map<String, List<HardwareDevice>> allDevicesMap = new HashMap<>(15);

    public void put(String deviceName, HardwareDevice device){
        deviceName = deviceName.trim();
        List<HardwareDevice> list = allDevicesMap.get(deviceName);
        if (list == null){
            list = new ArrayList<>(1);
            allDevicesMap.put(deviceName, list);
        }
        list.add(device);
        if (device instanceof DCMotor) dcMotor.put(deviceName, (DCMotor)device);
        if (device instanceof ColorSensor) colorSensor.put(deviceName, (ColorSensor)device);
        if (device instanceof GyroSensor) gyroSensor.put(deviceName, (GyroSensor)device);
        if (device instanceof Servo) servo.put(deviceName, (Servo)device);
    }

    public <T> T get(Class<? extends T> classOrInterface, String deviceName){
        T result = tryGet(classOrInterface, deviceName);
        if (result == null) throw new IllegalArgumentException(
                String.format("No %s named %s is found.", classOrInterface.getName(), deviceName)
        );
        return result;
    }

    private <T> T tryGet(Class<? extends T> classOrInterface, String deviceName){
        deviceName = deviceName.trim();
        List<HardwareDevice> list = allDevicesMap.get(deviceName);
        if (list != null) {
            for (HardwareDevice device : list){
                if(classOrInterface.isInstance(device)) return classOrInterface.cast(device);
            }
        }
        return null;
    }

    public class DeviceMapping<DEVICE_TYPE extends HardwareDevice>{

        private Map<String,DEVICE_TYPE> map = new HashMap<>();
        private Class<DEVICE_TYPE> deviceTypeClass;

        DeviceMapping(Class<DEVICE_TYPE> deviceTypeClass){
            this.deviceTypeClass = deviceTypeClass;
        }

        public DEVICE_TYPE get(String deviceName){
            deviceName = deviceName.trim();
            DEVICE_TYPE result = map.get(deviceName);
            if (result == null) throw new IllegalArgumentException(
                    String.format("No %s named %s is found.", deviceTypeClass.getName(), deviceName)
            );
            return result;
        }

        public void put(String deviceName, DEVICE_TYPE device){
            deviceName = deviceName.trim();
            map.put(deviceName, device);
        }
    }

}
