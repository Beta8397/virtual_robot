package controller;

import background.Background;
import hardware.*;
import javafx.application.Platform;
import javafx.event.ActionEvent;
import javafx.fxml.FXML;
import javafx.scene.Group;
import javafx.scene.control.ComboBox;
import javafx.scene.control.TextArea;
import javafx.scene.image.Image;
import javafx.scene.image.ImageView;
import javafx.scene.image.PixelReader;
import javafx.scene.input.MouseButton;
import javafx.scene.input.MouseEvent;
import javafx.scene.layout.StackPane;
import javafx.scene.paint.Color;
import javafx.scene.shape.Circle;
import opmode.LinearOpMode;
import javafx.scene.control.Button;
import opmodelist.OpModes;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

/**
 * Controller class for the robot simulation.
 */
public class VirtualRobotController {

    //User Interface
    @FXML private Group bot;
    @FXML private Button driverButton;
    @FXML private ComboBox<String> cbxOpModes;
    @FXML private TextArea txtTelemetry;
    @FXML Circle joyStickLeftHandle;
    @FXML Circle joyStickRightHandle;
    @FXML StackPane joyStickLeftPane;
    @FXML StackPane joyStickRightPane;
    @FXML Button btnX;
    @FXML Button btnY;
    @FXML Button btnA;
    @FXML Button btnB;
    @FXML ImageView imgViewBackground;

    //Virtual Hardware
    private static DCMotorImpl leftMotor = null;
    private static DCMotorImpl rightMotor = null;
    private static ColorSensorImpl colorSensor = null;
    private static GyroSensorImpl gyro = null;
    private static GamePad gamePad = new GamePad();

    //Background Image
    private Image backgroundImage = Background.background;
    private PixelReader pixelReader = backgroundImage.getPixelReader();

    //OpMode Control
    private static LinearOpMode opMode = null;
    private static volatile boolean opModeInitialized = false;
    private static volatile boolean opModeStarted = false;
    Thread opModeThread = null;

    //Virtual Robot Control Engine
    ScheduledExecutorService executorService = null;
    private final double TIMER_INTERVAL_MILLISECONDS = 33;

    //Virtual Robot State
    private static final double ROBOT_WIDTH = 65;
    private static final double WHEEL_CIRCUMFERENCE = 62.8;
    double leftTicks, rightTicks;
    double robotX, robotY, robotHeadingRadians;

    //Telemetry
    private static volatile String telemetryText;
    private static volatile boolean telemetryTextChanged = false;

    public void initialize() {
        initHardware();
        updateRobotDisplay();
        cbxOpModes.setItems(OpModes.opModes);
        cbxOpModes.setValue(cbxOpModes.getItems().get(0));
        imgViewBackground.setImage(backgroundImage);
    }

    @FXML
    private void handleDriverButtonAction(ActionEvent event){
        if (!opModeInitialized){
            if (!initLinearOpMode()) return;
            driverButton.setText("START");
            opModeInitialized = true;
            Runnable runOpMode = new Runnable() {
                @Override
                public void run() {
                    runOpModeAndCleanUp();
                }
            };
            opModeThread = new Thread(runOpMode);
            opModeThread.setDaemon(true);
            Runnable updateDisplay = new Runnable() {
                @Override
                public void run() {
                    updateRobotDisplay();
                    updateTelemetryDisplay();
                }
            };
            Runnable singleCycle = new Runnable() {
                @Override
                public void run() {
                    updateRobotState();
                    Platform.runLater(updateDisplay);
                }
            };
            executorService = Executors.newSingleThreadScheduledExecutor();
            executorService.scheduleAtFixedRate(singleCycle, 0, 33, TimeUnit.MILLISECONDS);
            opModeThread.start();
        }
        else if (!opModeStarted){
            driverButton.setText("STOP");
            opModeStarted = true;
        }
        else{
            driverButton.setText("INIT");
            opModeInitialized = false;
            opModeStarted = false;
            if (opModeThread.isAlive() && !opModeThread.isInterrupted()) opModeThread.interrupt();
            if (!executorService.isShutdown()) executorService.shutdown();
            try{
                opModeThread.join(500);
            } catch(InterruptedException exc) {
                Thread.currentThread().interrupt();
            }
            if (opModeThread.isAlive()) System.out.println("OpMode Thread Failed to Terminate.");
            leftMotor.setPower(0);
            rightMotor.setPower(0);
            resetGamePad();
        }
    }

    private void runOpModeAndCleanUp(){
        opMode.runOpMode();
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        if (!executorService.isShutdown()) executorService.shutdown();
        opModeInitialized = false;
        opModeStarted = false;
        Platform.runLater(new Runnable() {
            public void run() {
                driverButton.setText("INIT");
                resetGamePad();
            }
        });
    }

    @FXML
    private void handleFieldMouseClick(MouseEvent arg){
        if (opModeInitialized || opModeStarted) return;
        if (arg.getButton() == MouseButton.PRIMARY) {
            double argX = Math.max(37.5, Math.min(562.5, arg.getX()));
            double argY = Math.max(37.5, Math.min(562.5, arg.getY()));
            double displayX = argX - 37.5;
            double displayY = argY - 37.5;
            robotX = argX - 300.0;
            robotY = 300.0 - argY;
            bot.setTranslateX(displayX);
            bot.setTranslateY(displayY);
        }
        else if (arg.getButton() == MouseButton.SECONDARY){
            double centerX = robotX + 300;
            double centerY = 300 - robotY;
            double displayAngleRads = Math.atan2(arg.getY() - centerY, arg.getX() - centerX);
            double displayAngleDegrees = displayAngleRads * 180.0 / Math.PI;
            robotHeadingRadians = -displayAngleRads;
            bot.setRotate(displayAngleDegrees);
        }
    }

    @FXML
    private void handleJoystickDrag(MouseEvent arg){
        if (!opModeInitialized || !opModeStarted) return;
        float x = (float)Math.max(10, Math.min(110, arg.getX()));
        float y = (float)Math.max(10, Math.min(110, arg.getY()));
        if (arg.getSource() == joyStickLeftPane){
            joyStickLeftHandle.setTranslateX(x-10);
            joyStickLeftHandle.setTranslateY(y-10);
            gamePad.left_stick_x = (x - 60.0f) / 50.0f;
            gamePad.left_stick_y = (y - 60.0f) / 50.0f;
        }
        else if (arg.getSource() == joyStickRightPane){
            joyStickRightHandle.setTranslateX(x-10);
            joyStickRightHandle.setTranslateY(y-10);
            gamePad.right_stick_x = (x - 60.0f) / 50.0f;
            gamePad.right_stick_y = (y - 60.0f) / 50.0f;
        }
    }

    @FXML
    private void handleGamePadButtonMouseEvent(MouseEvent arg){
        if (!opModeInitialized || !opModeStarted) return;
        Button btn = (Button)arg.getSource();
        boolean result = false;

        if (arg.getEventType() == MouseEvent.MOUSE_EXITED || arg.getEventType() == MouseEvent.MOUSE_RELEASED) result = false;
        else if (arg.getEventType() == MouseEvent.MOUSE_PRESSED) result = true;
        else return;

        if (btn == btnX) gamePad.x = result;
        else if (btn == btnY) gamePad.y = result;
        else if (btn == btnA) gamePad.a = result;
        else if (btn == btnB) gamePad.b = result;
    }

    private void resetGamePad(){
        gamePad.left_stick_y = 0;
        gamePad.left_stick_x = 0;
        gamePad.right_stick_x = 0;
        gamePad.right_stick_y = 0;
        gamePad.a = false;
        gamePad.b = false;
        gamePad.x = false;
        gamePad.y = false;
        joyStickLeftHandle.setTranslateX(50);
        joyStickLeftHandle.setTranslateY(50);
        joyStickRightHandle.setTranslateX(50);
        joyStickRightHandle.setTranslateY(50);
    }


    private boolean initLinearOpMode(){
        String opModeName = cbxOpModes.getValue();
        opModeName = "teamcode." + opModeName;
        try {
            Class opModeClass = Class.forName(opModeName);
            opMode = (LinearOpMode)opModeClass.newInstance();
        } catch (Exception exc){
            return false;
        }
        return true;
    }

    private void initHardware(){
        leftMotor = new DCMotorImpl();
        rightMotor = new DCMotorImpl();
        colorSensor = new ColorSensorImpl();
        gyro = new GyroSensorImpl();
    }

    private synchronized void updateRobotState(){
        leftMotor.updatePosition(TIMER_INTERVAL_MILLISECONDS);
        rightMotor.updatePosition(TIMER_INTERVAL_MILLISECONDS);
        double newLeftTicks = leftMotor.getCurrentPositionDouble();
        double newRightTicks = rightMotor.getCurrentPositionDouble();
        double intervalLeftTicks = newLeftTicks - leftTicks;
        double intervalRightTicks = newRightTicks - rightTicks;
        leftTicks = newLeftTicks;
        rightTicks = newRightTicks;
        double leftWheelDist = intervalLeftTicks * WHEEL_CIRCUMFERENCE / DCMotorImpl.TICKS_PER_ROTATION;
        if (leftMotor.getDirection() == DCMotor.Direction.FORWARD) leftWheelDist = -leftWheelDist;
        double rightWheelDist = intervalRightTicks * WHEEL_CIRCUMFERENCE / DCMotorImpl.TICKS_PER_ROTATION;
        if (rightMotor.getDirection() == DCMotor.Direction.REVERSE) rightWheelDist = -rightWheelDist;
        double distTraveled = (leftWheelDist + rightWheelDist) / 2.0;
        double headingChange = (rightWheelDist - leftWheelDist) / ROBOT_WIDTH;
        double deltaRobotX = distTraveled * Math.cos(robotHeadingRadians + headingChange / 2.0);
        double deltaRobotY = distTraveled * Math.sin(robotHeadingRadians + headingChange / 2.0);
        robotX += deltaRobotX;
        robotY += deltaRobotY;
        if (robotX > 262.5 && deltaRobotX > 0) robotX = 262.5;
        else if (robotX < -262.5 && deltaRobotX < 0) robotX = -262.5;
        if (robotY > 262.5 && deltaRobotY > 0) robotY = 262.5;
        else if (robotY < -262.5 && deltaRobotY < 0) robotY = -262.5;
        robotHeadingRadians += headingChange;
        if (robotHeadingRadians > Math.PI) robotHeadingRadians -= 2.0 * Math.PI;
        else if (robotHeadingRadians < -Math.PI) robotHeadingRadians += 2.0 * Math.PI;
        gyro.updateHeading(robotHeadingRadians * 180.0 / Math.PI);
        int colorX = (int)(robotX + 300);
        int colorY = (int)(300 - robotY);
        double red = 0.0;
        double green = 0.0;
        double blue = 0.0;
        for (int row = colorY-4; row < colorY+5; row++)
            for (int col = colorX - 4; col < colorX+5; col++){
                Color c = pixelReader.getColor(col, row);
                red += c.getRed();
                green += c.getGreen();
                blue += c.getBlue();
            }
            red = Math.floor( red * 256.0 / 81.0 );
            if (red == 256) red = 255;
            green = Math.floor( green * 256.0 / 81.0 );
            if (green == 256) green = 255;
            blue = Math.floor( blue * 256.0 / 81.0 );
            if (blue == 256) blue = 255;
        colorSensor.updateColor((int)red, (int)green, (int)blue);
    }

    private synchronized void updateRobotDisplay(){
        double displayX = 300 + robotX - 37.5;
        double displayY = 300 - robotY - 37.5;
        double displayAngle = -robotHeadingRadians * 180.0 / Math.PI;
        bot.setTranslateX(displayX);
        bot.setTranslateY(displayY);
        bot.setRotate(displayAngle);
    }

    private void updateTelemetryDisplay(){
        if (telemetryTextChanged && telemetryText != null) txtTelemetry.setText(telemetryText);
        telemetryTextChanged = false;
    }

    private void timerCycle(){
        updateRobotState();
        updateRobotDisplay();
    }

    private class ColorSensorImpl implements ColorSensor {
        private int red = 0;
        private int green = 0;
        private int blue = 0;
        public synchronized int red(){ return red; }
        public synchronized int green(){ return green; }
        public synchronized int blue(){ return blue; }
        synchronized void updateColor(int red, int green, int blue){
            this.red = red;
            this.green = green;
            this. blue = blue;
        }
    }

    private class GyroSensorImpl implements GyroSensor {
        private boolean initialized = false;
        private double initialHeading = 0.0;
        private double heading = 0.0;
        public synchronized void init(){
            initialized = true;
            initialHeading = robotHeadingRadians * 180.0 / Math.PI;
        }
        synchronized void deinit(){
            initialized = false;
            initialHeading = 0.0;
            heading = 0.0;
        }
        public synchronized double getHeading(){
            if (initialized){
                double result = heading - initialHeading;
                if (result < -180.0) result += 360.0;
                else if (result > 180.0) result -= 360.0;
                return result;
            }
            else return 0.0;
        }
        synchronized void updateHeading(double heading){ this.heading = heading; }
    }

    private class DCMotorImpl implements DCMotor {
        private static final double MAX_TICKS_PER_SEC = 2500.0;
        private static final double TICKS_PER_ROTATION = 1120;
        private DCMotor.RunMode mode = RunMode.RUN_WITHOUT_ENCODER;
        private DCMotor.Direction direction = Direction.FORWARD;
        private double power = 0.0;
        private double position = 0.0;

        public synchronized void setMode(DCMotor.RunMode mode){
            this.mode = mode;
            if (mode == RunMode.STOP_AND_RESET_ENCODER){
                power = 0.0;
                position = 0.0;
            }
        }

        public synchronized DCMotor.RunMode getMode(){ return mode; }
        public synchronized void setDirection(DCMotor.Direction direction){ this.direction = direction; }
        public synchronized DCMotor.Direction getDirection(){ return direction; }
        public synchronized double getPower(){ return power; }
        public synchronized void setPower(double power){ this.power = power; }
        public synchronized int getCurrentPosition(){ return (int)Math.floor(position);}
        public synchronized double getCurrentPositionDouble(){ return position; }
        synchronized void updatePosition(double milliseconds){
            if (mode == RunMode.RUN_TO_POSITION || mode == RunMode.STOP_AND_RESET_ENCODER) return;
            position += power * MAX_TICKS_PER_SEC * milliseconds / 1000.0;
        }
    }

    private static class HardwareMapImpl implements HardwareMap{
        HardwareMapImpl(){
            dcMotor.put("left_motor", leftMotor);
            dcMotor.put("right_motor", rightMotor);
            colorSensor.put("color_sensor",VirtualRobotController.colorSensor);
            gyroSensor.put("gyro_sensor", gyro);
        }
    }


    /**
     * Base class for LinearOpMode.
     */
    public static class LinearOpModeBase {
        protected final HardwareMap hardwareMap;
        protected final GamePad gamePad1;
        protected final Telemetry telemetry;

        public LinearOpModeBase(){
            hardwareMap = new HardwareMapImpl();
            gamePad1 = gamePad;
            telemetry = new Telemetry();
            gyro.deinit();
        }

        /**
         * Pauses execution until the START button is pressed. Call this method after initialization code.
         */
        protected void waitForStart(){
            while (!opModeStarted) {
                try{
                    Thread.sleep(0);
                } catch (InterruptedException exc){
                    Thread.currentThread().interrupt();
                    return;
                }
            }
            return;
        }
    }

    public static class TelemetryBase {
        protected void setText(String text) {
            telemetryText = text;
            telemetryTextChanged = true;
        }
    }

}
