package virtual_robot.keyboard;

import javafx.scene.input.KeyCode;

import java.util.HashMap;

public class KeyState {
    private HashMap<KeyCode, Boolean> map = new HashMap<>();
    public boolean get(KeyCode code) { return map.get(code); }
    public void set(KeyCode code, boolean value) { map.put(code, value); }

    public KeyState(){
        for (KeyCode code : KeyCode.values()) map.put(code, false);
    }

}
