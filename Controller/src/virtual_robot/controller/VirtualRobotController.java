package virtual_robot.controller;

import javafx.beans.value.ChangeListener;
import javafx.beans.value.ObservableValue;
import javafx.collections.FXCollections;
import javafx.scene.control.*;
import virtual_robot.background.Background;
import virtual_robot.hardware.*;
import javafx.application.Platform;
import javafx.event.ActionEvent;
import javafx.fxml.FXML;
import javafx.geometry.Rectangle2D;
import javafx.scene.image.Image;
import javafx.scene.image.ImageView;
import javafx.scene.image.PixelReader;
import javafx.scene.input.MouseEvent;
import javafx.scene.layout.StackPane;
import javafx.scene.paint.Color;
import javafx.scene.shape.Circle;
import opmodelist.OpModes;
import java.util.Random;
import java.util.Set;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

/**
 * Controller class for the robot simulation.
 */
public class VirtualRobotController {

    //User Interface
    @FXML private StackPane fieldPane;
    @FXML ImageView imgViewBackground;
    @FXML private ComboBox<String> cbxConfig;
    @FXML private Button driverButton;
    @FXML private ComboBox<String> cbxOpModes;
    @FXML private Slider sldRandomMotorError;
    @FXML private Slider sldSystematicMotorError;
    @FXML private TextArea txtTelemetry;
    @FXML Circle joyStickLeftHandle;
    @FXML Circle joyStickRightHandle;
    @FXML StackPane joyStickLeftPane;
    @FXML StackPane joyStickRightPane;
    @FXML Button btnX;
    @FXML Button btnY;
    @FXML Button btnA;
    @FXML Button btnB;

    //Virtual Hardware
    private HardwareMapImpl hardwareMap = null;
    private VirtualBot bot = null;
    private GamePad gamePad = new GamePad();

    //Background Image and Field
    private Image backgroundImage = Background.background;
    private PixelReader pixelReader = backgroundImage.getPixelReader();
    private double halfFieldWidth;
    private double fieldWidth;

    //OpMode Control
    private LinearOpMode opMode = null;
    private volatile boolean opModeInitialized = false;
    private volatile boolean opModeStarted = false;
    private Thread opModeThread = null;

    //Virtual Robot Control Engine
    ScheduledExecutorService executorService = null;
    private final double TIMER_INTERVAL_MILLISECONDS = 33;

    //Telemetry
    private volatile String telemetryText;
    private volatile boolean telemetryTextChanged = false;

    //Random Number Generator
    private Random random = new Random();

    //Motor Error Slider Listener
    private ChangeListener<Number> sliderChangeListener = new ChangeListener<Number>() {
        @Override
        public void changed(ObservableValue<? extends Number> observable, Number oldValue, Number newValue) {
            for (DCMotorImpl motor: hardwareMap.dcMotor.values()) {
                motor.setRandomErrorFrac(sldRandomMotorError.getValue());
                motor.setSystematicErrorFrac(sldSystematicMotorError.getValue() * 2.0 * (0.5 - random.nextDouble()));
            }
        }
    };

    public void initialize() {
        LinearOpMode.setVirtualRobotController(this);
        cbxOpModes.setItems(OpModes.opModes);
        cbxOpModes.setValue(cbxOpModes.getItems().get(0));
        cbxConfig.setItems(FXCollections.observableArrayList("Two Wheel Bot", "Mechanum Bot"));
        cbxConfig.setValue(cbxConfig.getItems().get(0));
        fieldWidth = fieldPane.getPrefWidth();
        halfFieldWidth = fieldWidth / 2.0;
        fieldPane.setPrefHeight(fieldWidth);
        fieldPane.setMinWidth(fieldWidth);
        fieldPane.setMaxWidth(fieldWidth);
        fieldPane.setMinHeight(fieldWidth);
        fieldPane.setMaxHeight(fieldWidth);
        imgViewBackground.setFitWidth(fieldWidth);
        imgViewBackground.setFitHeight(fieldWidth);
        imgViewBackground.setViewport(new Rectangle2D(0, 0, fieldWidth, fieldWidth));
        imgViewBackground.setImage(backgroundImage);
        setConfig(null);
        initializeTelemetryTextArea();
        sldRandomMotorError.valueProperty().addListener(sliderChangeListener);
        sldSystematicMotorError.valueProperty().addListener(sliderChangeListener);
    }

    @FXML
    private void setConfig(ActionEvent event){
        if (opModeInitialized || opModeStarted) return;
        if (bot != null) bot.removeFromDisplay(fieldPane);
        if (cbxConfig.getValue().equals("Mechanum Bot")){
            hardwareMap = new HardwareMapImpl(new String[]{"back_left_motor", "front_left_motor",
                    "front_right_motor", "back_right_motor"});
            bot = new MechanumBot(hardwareMap, fieldWidth, fieldPane);
        } else {
            hardwareMap = new HardwareMapImpl(new String[]{"left_motor", "right_motor"});
            bot = new TwoWheelBot(hardwareMap, fieldWidth, fieldPane);
        }
        initializeTelemetryTextArea();
        sldRandomMotorError.setValue(0.0);
    }


    @FXML
    private void handleDriverButtonAction(ActionEvent event){
        if (!opModeInitialized){
            if (!initLinearOpMode()) return;
            txtTelemetry.setText("");
            driverButton.setText("START");
            opModeInitialized = true;
            cbxConfig.setDisable(true);
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
                    bot.updateDisplay();
                    updateTelemetryDisplay();
                }
            };
            Runnable singleCycle = new Runnable() {
                @Override
                public void run() {
                    bot.updateStateAndSensors(TIMER_INTERVAL_MILLISECONDS);
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
            bot.powerDownAndReset();
            resetGamePad();
            initializeTelemetryTextArea();
            cbxConfig.setDisable(false);
        }
    }

    private void runOpModeAndCleanUp(){
        opMode.runOpMode();
        bot.powerDownAndReset();
        if (!executorService.isShutdown()) executorService.shutdown();
        opModeInitialized = false;
        opModeStarted = false;
        Platform.runLater(new Runnable() {
            public void run() {
                driverButton.setText("INIT");
                resetGamePad();
                initializeTelemetryTextArea();
                cbxConfig.setDisable(false);
            }
        });
    }

    @FXML
    private void handleFieldMouseClick(MouseEvent arg){
        if (opModeInitialized || opModeStarted) return;
        bot.positionWithMouseClick(arg);
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
        boolean result;

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



    private void updateTelemetryDisplay(){
        if (telemetryTextChanged && telemetryText != null) txtTelemetry.setText(telemetryText);
        telemetryTextChanged = false;
    }

    private void initializeTelemetryTextArea(){
        StringBuilder sb = new StringBuilder();
        sb.append("Left-click to position bot.");
        sb.append("\nRight-click to orient bot.");
        sb.append("\n\nCONFIG");
        sb.append("\n Motors:");
        Set<String> motors = hardwareMap.dcMotor.keySet();
        for (String motor: motors) sb.append("\n   " + motor);
        sb.append("\n Servos:");
        Set<String> servos = hardwareMap.servo.keySet();
        for (String servo: servos) sb.append("\n   " + servo);
        sb.append("\n Color Sensors:");
        Set<String> colorSensors = hardwareMap.colorSensor.keySet();
        for (String colorSensor: colorSensors) sb.append("\n   " + colorSensor);
        sb.append("\n Gyro Sensors:");
        Set<String> gyroSensors = hardwareMap.gyroSensor.keySet();
        for (String gyroSensor: gyroSensors) sb.append("\n   " + gyroSensor);
        txtTelemetry.setText(sb.toString());
    }

    public class ColorSensorImpl implements ColorSensor {
        private int red = 0;
        private int green = 0;
        private int blue = 0;
        public synchronized int red(){ return red; }
        public synchronized int green(){ return green; }
        public synchronized int blue(){ return blue; }

        synchronized void updateColor(double x, double y){
            int colorX = (int)(x + halfFieldWidth);
            int colorY = (int)(halfFieldWidth - y);
            double tempRed = 0.0;
            double tempGreen = 0.0;
            double tempBlue = 0.0;
            for (int row = colorY-4; row < colorY+5; row++)
                for (int col = colorX - 4; col < colorX+5; col++){
                    Color c = pixelReader.getColor(col, row);
                    tempRed += c.getRed();
                    tempGreen += c.getGreen();
                    tempBlue += c.getBlue();
                }
            tempRed = Math.floor( tempRed * 256.0 / 81.0 );
            if (tempRed == 256) tempRed = 255;
            tempGreen = Math.floor( tempGreen * 256.0 / 81.0 );
            if (tempGreen == 256) tempGreen = 255;
            tempBlue = Math.floor( tempBlue * 256.0 / 81.0 );
            if (tempBlue == 256) tempBlue = 255;
            red = (int)tempRed;
            green = (int)tempGreen;
            blue = (int)tempBlue;
        }
    }

    public class GyroSensorImpl implements GyroSensor {
        private boolean initialized = false;
        private double initialHeading = 0.0;
        private double heading = 0.0;
        public synchronized void init(){
            initialized = true;
            initialHeading = bot.getHeadingRadians() * 180.0 / Math.PI;
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

    public class DCMotorImpl implements DCMotor {
        public static final double MAX_TICKS_PER_SEC = 2500.0;
        public static final double TICKS_PER_ROTATION = 1120;
        private DCMotor.RunMode mode = RunMode.RUN_WITHOUT_ENCODER;
        private DCMotor.Direction direction = Direction.FORWARD;
        private double power = 0.0;
        private double position = 0.0;
        private double randomErrorFrac = 0.0;
        private double systematicErrorFrac = 0.0;

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

        public synchronized void setPower(double power){
            this.power = Math.max(-1, Math.min(1, power));
        }

        public synchronized int getCurrentPosition(){ return (int)Math.floor(position);}
        synchronized double getCurrentPositionDouble(){ return position; }
        synchronized void updatePosition(double milliseconds){
            if (mode == RunMode.RUN_TO_POSITION || mode == RunMode.STOP_AND_RESET_ENCODER) return;
            double positionChange = power * MAX_TICKS_PER_SEC * milliseconds / 1000.0;
            positionChange *= (1.0 + systematicErrorFrac + randomErrorFrac * random.nextGaussian());
            position += positionChange;
        }

        synchronized void setRandomErrorFrac(double rdmErrFrac){
            randomErrorFrac = rdmErrFrac;
        }
        synchronized void setSystematicErrorFrac(double sysErrFrac) { systematicErrorFrac = sysErrFrac; }

    }

    public class ServoImpl implements Servo{
        private double position;

        public synchronized void setPosition(double position) {
            this.position = Math.max(0, Math.min(1, position));
        }

        public synchronized double getPosition(){
            return position;
        }
    }

    public class HardwareMapImpl implements HardwareMap{

        public HardwareMapImpl(){
            dcMotor.clear();
            dcMotor.put("left_motor", new DCMotorImpl());
            dcMotor.put("right_motor", new DCMotorImpl());
            colorSensor.clear();
            colorSensor.put("color_sensor", new ColorSensorImpl());
            gyroSensor.clear();
            gyroSensor.put("gyro_sensor", new GyroSensorImpl());
            servo.clear();
            servo.put("back_servo", new ServoImpl());
        }

        public HardwareMapImpl( String[] motors ){
            dcMotor.clear();
            if (motors != null) for (int i=0; i<motors.length; i++) dcMotor.put(motors[i], new DCMotorImpl());
            colorSensor.clear();
            colorSensor.put("color_sensor", new ColorSensorImpl());
            gyroSensor.clear();
            gyroSensor.put("gyro_sensor", new GyroSensorImpl());
            servo.clear();
            servo.put("back_servo", new ServoImpl());
        }


    }


    /**
     * Base class for LinearOpMode.
     */
    public class LinearOpModeBase {
        protected final HardwareMap hardwareMap;
        protected final GamePad gamePad1;
        protected final Telemetry telemetry;

        public LinearOpModeBase(){
            hardwareMap = VirtualRobotController.this.hardwareMap;
            gamePad1 = gamePad;
            telemetry = new TelemetryImpl();
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

    public class TelemetryImpl implements Telemetry {

        public TelemetryImpl(){
            update();
        }

        /**
         * Add data to telemetry (note-must call update() to cause the data to be displayed)
         * @param caption The caption for this telemetry entry.
         * @param fmt Format string, for formatting the data.
         * @param data The data to be formatted by the format string.
         */
        public void addData(String caption, String fmt, Object... data){
            this.data.append(caption + ": ");
            String s = String.format(fmt, data);
            this.data.append(s + "\n");
        }

        /**
         * Add single data object to telemetry, with a caption (note-must call update() to cause the data to be displayed)
         * @param caption The caption for this telemetry entry.
         * @param data The data for this telemetry entry.
         */
        public void addData(String caption, Object data){
            this.data.append(caption + ":" + data.toString() + "\n");
        }


        /**
         * Replace any data currently displayed on telemetry with all data that has been added since the previous call to
         * update().
         */
        public void update(){
            setText(data.toString());
            data.setLength(0);
        }

        private void setText(String text){
            telemetryText = text;
            telemetryTextChanged = true;
        }

    }

}
