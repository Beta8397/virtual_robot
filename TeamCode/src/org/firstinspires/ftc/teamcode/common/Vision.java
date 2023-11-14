package org.firstinspires.ftc.teamcode.common;


//import android.util.Size;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

/**
 * Component wrapper to support the v8.2+ SDK's included libraries for Camera operation.
 * This is an expansible system to run Processor components using the VisionPortal.
 *
 * @author Lucas Bubner, 2023
 */
public class Vision extends BunyipsComponent {
    public Vision(BunyipsOpMode o, WebcamName w) {
        super(o);
        // This class is No-op as sdk v9.0 has not been implemented upstream
    }
}