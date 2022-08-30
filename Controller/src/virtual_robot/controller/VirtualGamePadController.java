package virtual_robot.controller;

import com.qualcomm.robotcore.hardware.Gamepad;

import javafx.application.Platform;
import javafx.beans.value.ChangeListener;
import javafx.beans.value.ObservableValue;
import javafx.fxml.FXML;
import javafx.scene.control.Button;
import javafx.scene.control.Slider;
import javafx.scene.input.KeyCode;
import javafx.scene.input.KeyEvent;
import javafx.scene.input.MouseEvent;
import javafx.scene.layout.Background;
import javafx.scene.layout.HBox;
import javafx.scene.layout.StackPane;
import javafx.scene.shape.Circle;
import javafx.scene.shape.Rectangle;
import virtual_robot.config.Config;

import java.security.Key;
import java.util.ListIterator;

public class VirtualGamePadController {

    @FXML StackPane joyStickLeftPane;
    @FXML StackPane joyStickRightPane;
    @FXML Circle joyStickLeftHandle;
    @FXML
    Circle joyStickRightHandle;
    @FXML
    HBox gamepadBackground;
    @FXML
    Button btnX;
    @FXML Button btnY;
    @FXML Button btnA;
    @FXML Button btnB;
    @FXML Button btnDU;
    @FXML Button btnDL;
    @FXML Button btnDR;
    @FXML Button btnDD;
    @FXML Button btnLB;
    @FXML Button btnRB;
    @FXML Slider sldLeft;
    @FXML Slider sldRight;


    volatile float left_stick_x = 0;
    volatile float left_stick_y = 0;
    volatile float right_stick_x = 0;
    volatile float right_stick_y = 0;
    volatile boolean xPressed = false;
    volatile boolean yPressed = false;
    volatile boolean aPressed = false;
    volatile boolean bPressed = false;
    volatile boolean dUPressed = false;
    volatile boolean dLPressed = false;
    volatile boolean dRPressed = false;
    volatile boolean dDPressed = false;
    volatile boolean lBPressed = false;
    volatile boolean rBPressed = false;
    volatile float leftTrigger = 0;
    volatile float rightTrigger = 0;


    VirtualRobotController virtualRobotController = null;

    ChangeListener<Number> sliderChangeListener = new ChangeListener<Number>() {
        @Override
        public void changed(ObservableValue<? extends Number> observable, Number oldValue, Number newValue) {
            leftTrigger = (float)sldLeft.getValue();
            rightTrigger = (float)sldRight.getValue();
        }
    };

    public void initialize(){
        System.out.println("Initializing virtual gamepad");
        sldLeft.valueProperty().addListener(sliderChangeListener);
        sldRight.valueProperty().addListener(sliderChangeListener);
    }

    void setVirtualRobotController(VirtualRobotController vrController){
        virtualRobotController = vrController;
    }


    @FXML
    private void handleJoystickMouseEvent(MouseEvent arg){
        if (!virtualRobotController.getOpModeInitialized()) return;
        if (arg.getEventType() == MouseEvent.MOUSE_DRAGGED) {
            float x = (float) Math.max(10, Math.min(110, arg.getX()));
            float y = (float) Math.max(10, Math.min(110, arg.getY()));
            if (arg.getSource() == joyStickLeftPane) {
                joyStickLeftHandle.setTranslateX(x - 10);
                joyStickLeftHandle.setTranslateY(y - 10);
                left_stick_x = (x - 60.0f) / 50.0f;
                left_stick_y = (y - 60.0f) / 50.0f;
            } else if (arg.getSource() == joyStickRightPane) {
                joyStickRightHandle.setTranslateX(x - 10);
                joyStickRightHandle.setTranslateY(y - 10);
                right_stick_x = (x - 60.0f) / 50.0f;
                right_stick_y = (y - 60.0f) / 50.0f;
            }
        } else if (arg.getEventType() == MouseEvent.MOUSE_RELEASED){
            boolean keyDown = virtualRobotController.getKeyState(KeyCode.SHIFT)
                    || virtualRobotController.getKeyState(KeyCode.ALT);
            if ((keyDown && Config.HOLD_CONTROLS_BY_DEFAULT)
                    || (!keyDown && !Config.HOLD_CONTROLS_BY_DEFAULT)) {
                if (arg.getSource() == joyStickLeftPane) {
                    joyStickLeftHandle.setTranslateX(50);
                    joyStickLeftHandle.setTranslateY(50);
                    left_stick_x = 0;
                    left_stick_y = 0;
                } else if (arg.getSource() == joyStickRightPane) {
                    joyStickRightHandle.setTranslateX(50);
                    joyStickRightHandle.setTranslateY(50);
                    right_stick_x = 0;
                    right_stick_y = 0;
                }
            }
        }
    }

    @FXML
    public void handleTriggerMouseEvent(MouseEvent arg){
        if (arg.getEventType() == MouseEvent.MOUSE_RELEASED){
            boolean keyDown = virtualRobotController.getKeyState(KeyCode.SHIFT)
                    || virtualRobotController.getKeyState(KeyCode.ALT);
            if ((keyDown && Config.HOLD_CONTROLS_BY_DEFAULT)
                    || (!keyDown && !Config.HOLD_CONTROLS_BY_DEFAULT)) {
                ((Slider)arg.getSource()).setValue(0);
            }
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
        else if (btn == btnDU) dUPressed = result;
        else if (btn == btnDL) dLPressed = result;
        else if (btn == btnDR) dRPressed = result;
        else if (btn == btnDD) dDPressed = result;
        else if (btn == btnLB) lBPressed = result;
        else if (btn == btnRB) rBPressed = result;
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
        dUPressed = false;
        dLPressed = false;
        dRPressed = false;
        dDPressed = false;
        lBPressed = false;
        rBPressed = false;
        sldLeft.setValue(0);
        sldRight.setValue(0);
        leftTrigger = 0;
        rightTrigger = 0;
        joyStickLeftHandle.setTranslateX(50);
        joyStickLeftHandle.setTranslateY(50);
        joyStickRightHandle.setTranslateX(50);
        joyStickRightHandle.setTranslateY(50);

        interruptLEDandRumbleThreads();

        final String normalStyle = "-fx-background-color: #FFFFFF";
        if (Platform.isFxApplicationThread()){
            sldLeft.setStyle(normalStyle);
            sldRight.setStyle(normalStyle);
            gamepadBackground.setStyle(normalStyle);
        } else {
            Platform.runLater(new Runnable() {
                @Override
                public void run() {
                    sldLeft.setStyle(normalStyle);
                    sldRight.setStyle(normalStyle);
                    gamepadBackground.setStyle(normalStyle);
                }
            });
        }

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
        public final boolean dpad_up;
        public final boolean dpad_left;
        public final boolean dpad_right;
        public final boolean dpad_down;
        public final boolean left_bumper;
        public final boolean right_bumper;
        public final float left_trigger;
        public final float right_trigger;

        public ControllerState(){
            leftStickX = VirtualGamePadController.this.left_stick_x;
            leftStickY = VirtualGamePadController.this.left_stick_y;
            rightStickX = VirtualGamePadController.this.right_stick_x;
            rightStickY = VirtualGamePadController.this.right_stick_y;
            a = VirtualGamePadController.this.aPressed;
            b = VirtualGamePadController.this.bPressed;
            x = VirtualGamePadController.this.xPressed;
            y = VirtualGamePadController.this.yPressed;
            dpad_up = VirtualGamePadController.this.dUPressed;
            dpad_left = VirtualGamePadController.this.dLPressed;
            dpad_right = VirtualGamePadController.this.dRPressed;
            dpad_down = VirtualGamePadController.this.dDPressed;
            left_bumper = VirtualGamePadController.this.lBPressed;
            right_bumper = VirtualGamePadController.this.rBPressed;
            left_trigger = VirtualGamePadController.this.leftTrigger;
            right_trigger = VirtualGamePadController.this.rightTrigger;
        }
    }

    ControllerState getState() {
        return new ControllerState();
    }

    public class LEDPattern implements Runnable {
        Gamepad.LedEffect leds;
        HBox background;

        LEDPattern(Gamepad.LedEffect leds, HBox background) {
            this.leds = leds;
            this.background = background;
        }

        @Override
        public void run() {
            do {
                ListIterator<Gamepad.LedEffect.Step> stepIterator = leds.steps.listIterator();
                while (stepIterator.hasNext()) {
                    Gamepad.LedEffect.Step step = stepIterator.next();
                    final String styleString = String.format("-fx-background-color: #%02X%02X%02X", step.r, step.g, step.b);
                    Platform.runLater(new Runnable() {
                        @Override
                        public void run() {
                            gamepadBackground.setStyle(styleString);
                        }
                    });

                    if (step.duration == -1) {
                        return;
                    }
                    try {
                        Thread.sleep(step.duration);
                    } catch (InterruptedException e) {
                        return;  // don't know why it was interrupted, but lets just bail on this sequence
                    }
                }
            } while (leds.repeating);
        }
    }

    public class RumblePattern implements Runnable {
        Gamepad.RumbleEffect rumbles;
        Slider sldLeft;
        Slider sldRight;
        HBox background;

        RumblePattern(Gamepad.RumbleEffect rumbles, Slider sldLeft, Slider sldRight) {
            this.rumbles = rumbles;
            this.sldLeft = sldLeft;
            this.sldRight = sldRight;
        }

        @Override
        public void run() {
            ListIterator<Gamepad.RumbleEffect.Step> stepIterator = rumbles.steps.listIterator();
            while (stepIterator.hasNext()) {
                Gamepad.RumbleEffect.Step step = stepIterator.next();
                final String leftStyle = String.format("-fx-background-color: #%02X%02X%02X", 255 - step.large, 255 - step.large, 255 - step.large);
                final String rightStyle = String.format("-fx-background-color: #%02X%02X%02X", 255 - step.small, 255 - step.small, 255 - step.small);
                Platform.runLater(new Runnable() {
                    @Override
                    public void run() {
                        sldLeft.setStyle(leftStyle);
                        sldRight.setStyle(rightStyle);
                    }
                });

                if (step.duration == -1) {
                    return;
                }
                try {
                    Thread.sleep(step.duration);
                } catch (InterruptedException e) {
                    return;  // don't know why it was interrupted, but lets just bail on this sequence
                }
            }
            Platform.runLater(new Runnable() {
                @Override
                public void run() {
                    sldLeft.setStyle("-fx-background-color: #FFFFFF");
                    sldRight.setStyle("-fx-background-color: #FFFFFF");
                }
            });
        }
    }

    Thread ledThread;
    Thread rumbleThread;


    void setOutputs(Gamepad gamepad) {
        // First, handle LEDs
        Gamepad.LedEffect leds = gamepad.ledQueue.poll();
        if (leds != null) {
            if (this.ledThread != null) {
                this.ledThread.interrupt();
            }
            this.ledThread = new Thread(new LEDPattern(leds, gamepadBackground));
            this.ledThread.start();
        }
        // Second, handle rumbles
        Gamepad.RumbleEffect rumbles = gamepad.rumbleQueue.poll();
        if (rumbles != null) {
            if (this.rumbleThread != null) {
                this.rumbleThread.interrupt();
            }
            // For our rumbles, we will set the background color of sldLeft and sldRight
            this.rumbleThread = new Thread(new RumblePattern(rumbles, sldLeft, sldRight));
            this.rumbleThread.start();
        }
    }

    void interruptLEDandRumbleThreads(){
        if (ledThread != null){
            ledThread.interrupt();
        }
        if (rumbleThread != null){
            rumbleThread.interrupt();
        }
    }
}
