package virtual_robot.controller;

import javafx.fxml.FXML;
import javafx.scene.control.Button;
import javafx.scene.input.MouseEvent;
import javafx.scene.layout.StackPane;
import javafx.scene.shape.Circle;

public class VirtualGamePadController {

    @FXML StackPane joyStickLeftPane;
    @FXML StackPane joyStickRightPane;
    @FXML Circle joyStickLeftHandle;
    @FXML Circle joyStickRightHandle;
    @FXML Button btnX;
    @FXML Button btnY;
    @FXML Button btnA;
    @FXML Button btnB;

    volatile float left_stick_x = 0;
    volatile float left_stick_y = 0;
    volatile float right_stick_x = 0;
    volatile float right_stick_y = 0;
    volatile boolean xPressed = false;
    volatile boolean yPressed = false;
    volatile boolean aPressed = false;
    volatile boolean bPressed = false;

    VirtualRobotController virtualRobotController = null;

    void setVirtualRobotController(VirtualRobotController vrController){
        virtualRobotController = vrController;
    }

    @FXML
    private void handleJoystickDrag(MouseEvent arg){
        if (!virtualRobotController.getOpModeInitialized()) return;
        float x = (float)Math.max(10, Math.min(110, arg.getX()));
        float y = (float)Math.max(10, Math.min(110, arg.getY()));
        if (arg.getSource() == joyStickLeftPane){
            joyStickLeftHandle.setTranslateX(x-10);
            joyStickLeftHandle.setTranslateY(y-10);
            left_stick_x = (x - 60.0f) / 50.0f;
            left_stick_y = (y - 60.0f) / 50.0f;
        }
        else if (arg.getSource() == joyStickRightPane){
            joyStickRightHandle.setTranslateX(x-10);
            joyStickRightHandle.setTranslateY(y-10);
            right_stick_x = (x - 60.0f) / 50.0f;
            right_stick_y = (y - 60.0f) / 50.0f;
        }
    }

    @FXML
    private void handleGamePadButtonMouseEvent(MouseEvent arg){
        if (!virtualRobotController.getOpModeInitialized()) return;
        Button btn = (Button)arg.getSource();
        boolean result;

        if (arg.getEventType() == MouseEvent.MOUSE_EXITED || arg.getEventType() == MouseEvent.MOUSE_RELEASED) result = false;
        else if (arg.getEventType() == MouseEvent.MOUSE_PRESSED) result = true;
        else return;

        if (btn == btnX) xPressed = result;
        else if (btn == btnY) yPressed = result;
        else if (btn == btnA) aPressed = result;
        else if (btn == btnB) bPressed = result;
    }

    void resetGamePad(){
        left_stick_y = 0;
        left_stick_x = 0;
        right_stick_x = 0;
        right_stick_y = 0;
        aPressed = false;
        bPressed = false;
        xPressed = false;
        yPressed = false;
        joyStickLeftHandle.setTranslateX(50);
        joyStickLeftHandle.setTranslateY(50);
        joyStickRightHandle.setTranslateX(50);
        joyStickRightHandle.setTranslateY(50);
    }

    public class ControllerState {

        public final float leftStickX;
        public final float leftStickY;
        public final float rightStickX;
        public final float rightStickY;
        public final boolean a;
        public final boolean b;
        public final boolean x;
        public final boolean y;

        public ControllerState(){
            leftStickX = VirtualGamePadController.this.left_stick_x;
            leftStickY = VirtualGamePadController.this.left_stick_y;
            rightStickX = VirtualGamePadController.this.right_stick_x;
            rightStickY = VirtualGamePadController.this.right_stick_y;
            a = VirtualGamePadController.this.aPressed;
            b = VirtualGamePadController.this.bPressed;
            x = VirtualGamePadController.this.xPressed;
            y = VirtualGamePadController.this.yPressed;
        }
    }

    ControllerState getState(){
        return new ControllerState();
    }

}
