package com.qualcomm.hardware.rev;

import com.qualcomm.robotcore.hardware.HardwareDevice;

public class RevBlinkinLedDriver implements HardwareDevice {
    public void resetDeviceConfigurationForOpMode() {
    }

    public void setPattern(BlinkinPattern pattern) {
    }

    public enum BlinkinPattern {BLACK, GREEN, WHITE, RED;

    }
}
