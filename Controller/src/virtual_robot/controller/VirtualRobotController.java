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
import opmodelist.OpModes;
import virtual_robot.hardware.bno055.BNO055IMU;
import virtual_robot.hardware.dcmotor.DcMotorImpl;
import virtual_robot.util.navigation.DistanceUnit;

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

    //Virtual Hardware
    private HardwareMap hardwareMap = null;
    private VirtualBot bot = null;
    GamePad gamePad = new GamePad();

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
            for (DcMotor motor: hardwareMap.dcMotor) {
                ((DcMotorImpl)motor).setRandomErrorFrac(sldRandomMotorError.getValue());
                ((DcMotorImpl)motor).setSystematicErrorFrac(sldSystematicMotorError.getValue() * 2.0 * (0.5 - random.nextDouble()));
            }
        }
    };

    public void initialize() {
        LinearOpMode.setVirtualRobotController(this);
        cbxOpModes.setItems(OpModes.opModes);
        cbxOpModes.setValue(cbxOpModes.getItems().get(0));
        cbxConfig.setItems(FXCollections.observableArrayList("Two Wheel Bot", "Mechanum Bot", "XDrive Bot"));
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
        sldRandomMotorError.valueProperty().addListener(sliderChangeListener);
        sldSystematicMotorError.valueProperty().addListener(sliderChangeListener);
    }

    @FXML
    public void setConfig(ActionEvent event){
        if (opModeInitialized || opModeStarted) return;
        if (bot != null) bot.removeFromDisplay(fieldPane);
        if (cbxConfig.getValue().equals("Mechanum Bot")){
            bot = new MechanumBot(this);
        } else if (cbxConfig.getValue().equals("Two Wheel Bot")){
            bot = new TwoWheelBot(this);
        } else {
            bot = new XDriveBot(this);
        }
        hardwareMap = bot.getHardwareMap();
        initializeTelemetryTextArea();
        sldRandomMotorError.setValue(0.0);
        sldSystematicMotorError.setValue(0.0);
    }

    public StackPane getFieldPane(){ return fieldPane; }

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
                    gamePad.update();
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
                //resetGamePad();
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
        Set<String> motors = hardwareMap.dcMotor.keySet();
        if (!motors.isEmpty()) {
            sb.append("\n Motors:");
            for (String motor : motors) sb.append("\n   " + motor);
        }
        Set<String> servos = hardwareMap.servo.keySet();
        if (!servos.isEmpty()) {
            sb.append("\n Servos:");
            for (String servo : servos) sb.append("\n   " + servo);
        }
        Set<String> colorSensors = hardwareMap.colorSensor.keySet();
        if (!colorSensors.isEmpty()) {
            sb.append("\n Color Sensors:");
            for (String colorSensor : colorSensors) sb.append("\n   " + colorSensor);
        }
        Set<String> gyroSensors = hardwareMap.gyroSensor.keySet();
        if (!gyroSensors.isEmpty()) {
            sb.append("\n Gyro Sensors:");
            for (String gyroSensor : gyroSensors) sb.append("\n   " + gyroSensor);
        }
        Set<String> bno055IMUs = hardwareMap.keySet(BNO055IMU.class);
        if (!bno055IMUs.isEmpty()){
            sb.append("\n BNO055IMU Sensors:");
            for (String imuSensor : bno055IMUs) sb.append("\n   " + imuSensor);
        }
        Set<String> distanceSensors = hardwareMap.keySet(DistanceSensor.class);
        if (!distanceSensors.isEmpty()) {
            sb.append("\n Distance Sensors:");
            for (String distance : distanceSensors) sb.append("\n   " + distance);
        }
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
            heading = initialHeading = bot.getHeadingRadians() * 180.0 / Math.PI;
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

    public class DistanceSensorImpl implements DistanceSensor {
        private double distanceMM = distanceOutOfRange;
        private static final double MIN_DISTANCE = 50; //mm
        private static final double MAX_DISTANCE = 1000; //mm
        private static final double MAX_OFFSET = 7.0 * Math.PI / 180.0;

        public synchronized double getDistance(DistanceUnit distanceUnit){
            double result;
            if (distanceMM < MIN_DISTANCE) result = MIN_DISTANCE - 1.0;
            else if (distanceMM > MAX_DISTANCE) result = distanceOutOfRange;
            else result = distanceMM;
            switch(distanceUnit){
                case METER:
                    return result / 1000.0;
                case CM:
                    return result / 10.0;
                case MM:
                    return result;
                case INCH:
                    return result / 25.4;
                default:
                    return result;
            }
        }

        public synchronized void updateDistance(double x, double y, double headingRadians){
            final double mmPerPixel = 144.0 * 25.4 / fieldWidth;
            final double piOver2 = Math.PI / 2.0;
            double temp = headingRadians / piOver2;
            int side = (int)Math.round(temp); //-2, -1 ,0, 1, or 2 (2 and -2 both refer to the right side)
            double offset = Math.abs(headingRadians - (side * Math.PI / 2.0));
            if (offset > MAX_OFFSET) distanceMM = distanceOutOfRange;
            else switch (side){
                case 2:
                case -2:
                    distanceMM = (y + halfFieldWidth) * mmPerPixel;
                    break;
                case -1:
                    distanceMM = (halfFieldWidth - x) * mmPerPixel;
                    break;
                case 0:
                    distanceMM = (halfFieldWidth - y) * mmPerPixel;
                    break;
                case 1:
                    distanceMM = (x + halfFieldWidth) * mmPerPixel;
                    break;
            }
        }
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


    /**
     * Base class for LinearOpMode.
     */
    public class LinearOpModeBase {
        protected final HardwareMap hardwareMap;
        protected final GamePad gamepad1;
        protected final Telemetry telemetry;

        public LinearOpModeBase(){
            hardwareMap = VirtualRobotController.this.hardwareMap;
            gamepad1 = gamePad;
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

        /**
         * Returns true IF: start button has been pressed AND there has been no request to stop.
         *
         * This method also gives other threads an opportunity to run.
         *
         * Any long-running loop (more than a few iterations) should include a call to this method.
         *
         * @return TRUE if STARTED AND NO request to terminate; FALSE if there is a request to terminate or if not started.
         */
        protected boolean opModeIsActive(){
            if (Thread.currentThread().isInterrupted() || !opModeStarted) return false;
            try{
                Thread.sleep(0);
            } catch (InterruptedException exc){
                Thread.currentThread().interrupt();
                return false;
            }
            return true;
        }

        /**
         * Returns true if there has been request to stop op mode.
         *
         * @return TRUE if request to terminate; FALSE otherwise.
         */
        protected boolean isStopRequested(){
            if (Thread.currentThread().isInterrupted()) return true;
            return false;
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
