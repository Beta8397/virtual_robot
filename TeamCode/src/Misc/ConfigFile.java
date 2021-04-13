package Misc;

import com.qualcomm.robotcore.hardware.HardwareMap;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.InputStream;

public class ConfigFile {
    public static InputStream open(HardwareMap hardwareMap, String fileLoc) throws FileNotFoundException {
        return new FileInputStream(new File("TeamCode/assets/" + fileLoc));
    }
}
