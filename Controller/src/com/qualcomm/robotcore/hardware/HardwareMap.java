package com.qualcomm.robotcore.hardware;

import java.util.*;

/**
 * HardwareMap provides access to the virtual robot hardware
 */
public class HardwareMap {

    /**
     * Map of all DcMotor devices in this HardwareMap.
     */
    public final DeviceMapping<DcMotor> dcMotor = new DeviceMapping<>(DcMotor.class);

    /**
     * Map of all ColorSensor devices in this HardwareMap.
     */
    public final DeviceMapping<ColorSensor> colorSensor = new DeviceMapping<>(ColorSensor.class);

    /**
     * Map of all GyroSensor devices in this HardwareMap.
     */
    public final DeviceMapping<GyroSensor> gyroSensor = new DeviceMapping<>(GyroSensor.class);

    /**
     * Map of all Servo devices in this HardwareMap.
     */
    public final DeviceMapping<Servo> servo = new DeviceMapping<>(Servo.class);

    /**
     * Map of all CRServo devices in this HardwareMap.
     */
    public final DeviceMapping<CRServo> crservo = new DeviceMapping(CRServo.class);

    /**
     * Map of all hardware devices in this HardwareMap
     */
    private Map<String, List<HardwareDevice>> allDevicesMap = new HashMap<>(15);

    private final Object lock = new Object();

    /**
     * Add a device to the HardwareMap
     * @param deviceName name of device
     * @param device device object
     */
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

    /**
     * Obtain a device with a specified name and class or interface from the HardwareMap.
     * @param classOrInterface
     * @param deviceName
     * @param <T>
     * @return
     */
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

    /**
     * For internal use only.
     * Obtain set containing the names of all devices in the HardwareMap of a specified class or interface.
     * @param classOrInterface
     * @param <T>
     * @return
     */
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

    /**
     * Mapping of devices of a specified type.
     * @param <DEVICE_TYPE>
     */
    public class DeviceMapping<DEVICE_TYPE extends HardwareDevice> implements Iterable<DEVICE_TYPE>{

        private Map<String,DEVICE_TYPE> map = new HashMap<>();
        private Class<DEVICE_TYPE> deviceTypeClass;

        DeviceMapping(Class<DEVICE_TYPE> deviceTypeClass){
            this.deviceTypeClass = deviceTypeClass;
        }

        /**
         * Obtain device with type DEVICE_TYPE, with the specified device name.
         * @param deviceName
         * @return
         */
        public synchronized DEVICE_TYPE get(String deviceName){
            deviceName = deviceName.trim();
            DEVICE_TYPE result = map.get(deviceName);
            if (result == null) throw new IllegalArgumentException(
                    String.format("No %s named %s is found.", deviceTypeClass.getName(), deviceName)
            );
            return result;
        }

        /**
         * For internal use only.
         * @param deviceName
         * @param device
         */
        public synchronized void put(String deviceName, DEVICE_TYPE device){
            deviceName = deviceName.trim();
            map.put(deviceName, device);
        }

        /**
         * For internal use only.
         * @return
         */
        public synchronized Iterator<DEVICE_TYPE> iterator(){
            return new ArrayList<>(map.values()).iterator();
        }

        /**
         * For internal use only.
         * @return
         */
        public synchronized Set<String> keySet(){
            return map.keySet();
        }

    }

}
