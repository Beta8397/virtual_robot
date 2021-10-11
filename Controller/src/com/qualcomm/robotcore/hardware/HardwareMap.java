/* Copyright (c) 2014, 2015 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

/*
Modified by FTC Team Beta 8397 for use in the Virtual_Robot Simulator
 */

package com.qualcomm.robotcore.hardware;

import java.util.*;
import android.content.Context;

/**
 * HardwareMap provides access to the virtual robot hardware
 */
public class HardwareMap implements Iterable<HardwareDevice>{

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

    /**
     *  List of all hardware devices in this HardwareMap
     */
    private List<HardwareDevice> allDevicesList = new ArrayList<>();

    public Context appContext;

    /**
     * INTERNAL USE ONLY!!!
     * This is needed to prevent users from obtaining hardware references before the INIT button is pressed.
     * i.e., references to hardware should be obtained in the opmode.init(), opmode.loop(), or linearopmode.runopmode()
     * methods.
     */
    private boolean active = false;
    public void setActive(boolean isActive){ active = isActive; }

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
        allDevicesList.add(device);
        if (device instanceof DcMotor) dcMotor.put(deviceName, (DcMotor)device);
        if (device instanceof ColorSensor) colorSensor.put(deviceName, (ColorSensor)device);
        if (device instanceof GyroSensor) gyroSensor.put(deviceName, (GyroSensor)device);
        if (device instanceof Servo) servo.put(deviceName, (Servo)device);
        if (device instanceof CRServo) crservo.put(deviceName, (CRServo)device);
    }


    /**
     * (Advanced) Removes a device from the overall map, if present. If the device is also present in a
     * DeviceMapping, then the device should be removed using {@link DeviceMapping#remove}
     * instead of calling this method.
     *
     * <p>This is normally called only by code in the SDK itself, not by user code.</p>
     *
     * * @param deviceName  the name of the device to remove
     * @param device      the device to remove under that name
     * @return whether a device was removed or not
     */
    public boolean remove(String deviceName, HardwareDevice device) {
        deviceName = deviceName.trim();
        List<HardwareDevice> list = allDevicesMap.get(deviceName);
        if (list != null) {
            list.remove(device);
            if (list.isEmpty()) {
                allDevicesMap.remove(deviceName);
            }
            allDevicesList.remove(device);
            return true;
        }
        return false;
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
        if (!active){
            System.out.println("ERROR: Cannot obtain references to hardware before INIT button is pressed.");
            return null;
        }
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
     * Returns all the devices which are instances of the indicated class or interface.
     * @param classOrInterface the class or interface indicating the type of the device object to be retrieved
     * @return all the devices registered in the map which are instances of classOrInterface
     */
    public <T> List<T> getAll(Class<? extends T> classOrInterface) {
        List<T> result = new ArrayList<T>();
        for (HardwareDevice device : this) {
            if (classOrInterface.isInstance(device)) {
                result.add(classOrInterface.cast(device));
            }
        }
        return result;
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

    @Override
    public Iterator<HardwareDevice> iterator() {
        return allDevicesList.iterator();
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
            if (!active){
                System.out.println("ERROR: Cannot obtain references to hardware before INIT button is pressed.");
                return null;
            }
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
         * FOR INTERNAL USE ONLY
         *
         * (Advanced) Removes the device with the indicated name (if any) from this DeviceMapping. The device
         * is also removed under that name in the overall map itself. Note that this method is normally
         * called only by code in the SDK itself, not by user code.
         *
         * @param deviceName the name of the device to remove.
         * @return whether any modifications were made to this DeviceMapping
         * @see HardwareMap#remove
         */
        public synchronized boolean remove(String deviceName) {
            deviceName = deviceName.trim();
            HardwareDevice device = map.remove(deviceName);
            if (device != null) {
                HardwareMap.this.remove(deviceName, device);
                return true;
            }
            return false;
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
