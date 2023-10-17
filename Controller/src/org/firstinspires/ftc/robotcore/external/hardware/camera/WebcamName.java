package org.firstinspires.ftc.robotcore.external.hardware.camera;

import com.qualcomm.robotcore.hardware.HardwareDevice;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;

public class WebcamName implements CameraName, HardwareDevice {
    @Override
    public boolean isWebcam() {
        return false;
    }

    @Override
    public boolean isCameraDirection() {
        return false;
    }

    @Override
    public boolean isSwitchable() {
        return false;
    }

    @Override
    public boolean isUnknown() {
        return false;
    }

    @Override
    public boolean requestCameraPermission(Deadline deadline) {
        return false;
    }
}
