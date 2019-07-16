package com.qualcomm.robotcore.hardware;

import java.util.*;

/**
 * HardwareMap provides access to the virtual robot hardware
 */
public class HardwareMap {

    public final DeviceMapping<DcMotor> dcMotor = new DeviceMapping<>(DcMotor.class);
    public final DeviceMapping<ColorSensor> colorSensor = new DeviceMapping<>(ColorSensor.class);
    public final DeviceMapping<GyroSensor> gyroSensor = new DeviceMapping<>(GyroSensor.class);
    public final DeviceMapping<Servo> servo = new DeviceMapping<>(Servo.class);
    public final DeviceMapping<CRServo> crservo = new DeviceMapping(CRServo.class);

    private Map<String, List<HardwareDevice>> allDevicesMap = new HashMap<>(15);

    private final Object lock = new Object();

    public synchronized void put(String deviceName, HardwareDevice device){
        deviceName = deviceName.trim();
        List<HardwareDevice> list = allDevicesMap.get(deviceName);
        if (list == null){
            list = new ArrayList<>(1);
            allDevicesMap.put(deviceName, list);
        }
        list.add(device);
        if (device instanceof DcMotor) dcMotor.put(deviceName, (DcMotor)device);
        if (device instanceof ColorSensor) colorSensor.put(deviceName, (ColorSensor)device);
        if (device instanceof GyroSensor) gyroSensor.put(deviceName, (GyroSensor)device);
        if (device instanceof Servo) servo.put(deviceName, (Servo)device);
        if (device instanceof CRServo) crservo.put(deviceName, (CRServo)device);
    }

    public <T> T get(Class<? extends T> classOrInterface, String deviceName){
        T result = tryGet(classOrInterface, deviceName);
        if (result == null) throw new IllegalArgumentException(
                String.format("No %s named %s is found.", classOrInterface.getName(), deviceName)
        );
        return result;
    }

    private synchronized <T> T tryGet(Class<? extends T> classOrInterface, String deviceName){
        deviceName = deviceName.trim();
        List<HardwareDevice> list = allDevicesMap.get(deviceName);
        if (list != null) {
            for (HardwareDevice device : list){
                if(classOrInterface.isInstance(device)) return classOrInterface.cast(device);
            }
        }
        return null;
    }

    public synchronized <T> Set<String> keySet(Class<? extends T> classOrInterface){
        Set<String> result = new HashSet<>();
        for (String deviceName : allDevicesMap.keySet()){
            for (HardwareDevice device : allDevicesMap.get(deviceName)){
                if (classOrInterface.isInstance(device))result.add(deviceName);
                break;
            }
        }
        return result;
    }

    public class DeviceMapping<DEVICE_TYPE extends HardwareDevice> implements Iterable<DEVICE_TYPE>{

        private Map<String,DEVICE_TYPE> map = new HashMap<>();
        private Class<DEVICE_TYPE> deviceTypeClass;

        DeviceMapping(Class<DEVICE_TYPE> deviceTypeClass){
            this.deviceTypeClass = deviceTypeClass;
        }

        public synchronized DEVICE_TYPE get(String deviceName){
            deviceName = deviceName.trim();
            DEVICE_TYPE result = map.get(deviceName);
            if (result == null) throw new IllegalArgumentException(
                    String.format("No %s named %s is found.", deviceTypeClass.getName(), deviceName)
            );
            return result;
        }

        public synchronized void put(String deviceName, DEVICE_TYPE device){
            deviceName = deviceName.trim();
            map.put(deviceName, device);
        }

        public synchronized Iterator<DEVICE_TYPE> iterator(){
            return new ArrayList<>(map.values()).iterator();
        }

        public synchronized Set<String> keySet(){
            return map.keySet();
        }

    }

}
