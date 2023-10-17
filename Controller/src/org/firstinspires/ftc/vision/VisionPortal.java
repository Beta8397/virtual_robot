package org.firstinspires.ftc.vision;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;

public class VisionPortal {
    public static VisionPortal easyCreateWithDefaults(CameraName cameraName, VisionProcessor... processors)
    {
        return new VisionPortal();
    }

    public void stopStreaming(){

    }
}
