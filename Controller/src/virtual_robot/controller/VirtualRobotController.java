package virtual_robot.controller;

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.*;
import com.studiohartman.jamepad.ControllerManager;
import com.studiohartman.jamepad.ControllerState;
import javafx.beans.value.ChangeListener;
import javafx.beans.value.ObservableValue;
import javafx.collections.FXCollections;
import javafx.collections.ObservableList;
import javafx.fxml.FXMLLoader;
import javafx.scene.Group;
import javafx.scene.control.*;
import javafx.scene.layout.BorderPane;
import javafx.scene.layout.HBox;
import javafx.scene.shape.Polyline;
import javafx.scene.shape.Rectangle;
import javafx.util.Callback;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.reflections.Reflections;
import virtual_robot.config.Config;
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
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotorImpl;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import virtual_robot.controller.robots.classes.MechanumBot;
import virtual_robot.controller.robots.classes.TwoWheelBot;

import java.io.IOException;
import java.lang.annotation.Annotation;
import java.net.URL;
import java.util.*;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

/**
 * For internal use only. Controller class for the JavaFX application.
 */
public class VirtualRobotController {

    //User Interface
    @FXML private StackPane fieldPane;
    @FXML ImageView imgViewBackground;
    @FXML private ComboBox<Class<?>> cbxConfig;
    @FXML private Button driverButton;
    @FXML private ComboBox<Class<?>> cbxOpModes;
    @FXML private Slider sldRandomMotorError;
    @FXML private Slider sldSystematicMotorError;
    @FXML private Slider sldMotorInertia;
    @FXML private TextArea txtTelemetry;
    @FXML private CheckBox checkBoxGamePad1;
    @FXML private CheckBox checkBoxGamePad2;
    @FXML private BorderPane borderPane;
    @FXML private CheckBox cbxShowPath;

    //Virtual Hardware
    private HardwareMap hardwareMap = null;
    private VirtualBot bot = null;
    GamePad gamePad1 = new GamePad();
    GamePad gamePad2 = new GamePad();
    GamePadHelper gamePadHelper = null;
    ScheduledExecutorService gamePadExecutorService = Executors.newSingleThreadScheduledExecutor();

    VirtualGamePadController virtualGamePadController = null;

    //Background Image and Field
    private Image backgroundImage = Config.BACKGROUND;
    private PixelReader pixelReader = backgroundImage.getPixelReader();
    private double halfFieldWidth;
    private double fieldWidth;

    //Path Drawing
    Polyline pathLine;

    //Lists of OpMode classes and OpMode Names
    private ObservableList<Class<?>> nonDisabledOpModeClasses = null;

    //OpMode Control
    private OpMode opMode = null;
    private volatile boolean opModeInitialized = false;
    private volatile boolean opModeStarted = false;
    private Thread opModeThread = null;

    //Virtual Robot Control Engine
    ScheduledExecutorService executorService = null;
    public static final double TIMER_INTERVAL_MILLISECONDS = 33;

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
                ((DcMotorImpl)motor).setInertia(1.0 - Math.pow(10.0, -sldMotorInertia.getValue()));
            }
        }
    };

    boolean getOpModeInitialized(){ return opModeInitialized; }

    public void initialize() {
        OpMode.setVirtualRobotController(this);
        VirtualBot.setController(this);
        setupCbxOpModes();
        setupCbxRobotConfigs();
        fieldWidth = Config.FIELD_WIDTH;
        halfFieldWidth = fieldWidth / 2.0;
        fieldPane.setPrefWidth(fieldWidth);
        fieldPane.setPrefHeight(fieldWidth);
        fieldPane.setMinWidth(fieldWidth);
        fieldPane.setMaxWidth(fieldWidth);
        fieldPane.setMinHeight(fieldWidth);
        fieldPane.setMaxHeight(fieldWidth);
        imgViewBackground.setFitWidth(fieldWidth);
        imgViewBackground.setFitHeight(fieldWidth);
        imgViewBackground.setViewport(new Rectangle2D(0, 0, fieldWidth, fieldWidth));
        imgViewBackground.setImage(backgroundImage);
        Rectangle pathRect = new Rectangle(fieldWidth, fieldWidth);
        pathRect.setFill(Color.color(1,0,0,0));
        pathLine = new Polyline();
        pathLine.setStroke(Color.LAWNGREEN);
        pathLine.setStrokeWidth(2);
        pathLine.setVisible(false);
        fieldPane.getChildren().addAll(new Group(pathRect, pathLine));
        sldRandomMotorError.valueProperty().addListener(sliderChangeListener);
        sldSystematicMotorError.valueProperty().addListener(sliderChangeListener);
        sldMotorInertia.valueProperty().addListener(sliderChangeListener);
        if (Config.USE_VIRTUAL_GAMEPAD){
            checkBoxGamePad1.setVisible(false);
            checkBoxGamePad2.setVisible(false);
            FXMLLoader loader = new FXMLLoader(getClass().getResource("virtual_gamepad.fxml"));
            try{
                HBox hbox = (HBox)loader.load();
                virtualGamePadController = loader.getController();
                virtualGamePadController.setVirtualRobotController(this);
                borderPane.setBottom(hbox);
            } catch (IOException e){
                System.out.println("Virtual GamePad UI Failed to Load");
            }
            gamePadHelper = new VirtualGamePadHelper();
        } else {
            checkBoxGamePad1.setDisable(true);
            checkBoxGamePad1.setStyle("-fx-opacity: 1");
            checkBoxGamePad2.setDisable(true);
            checkBoxGamePad2.setStyle("-fx-opacity: 1");
            gamePadHelper = new RealGamePadHelper();
        }
        gamePadExecutorService.scheduleAtFixedRate(gamePadHelper, 0, 20, TimeUnit.MILLISECONDS);
    }

    private void setupCbxRobotConfigs(){
        //Reflections reflections = new Reflections(VirtualRobotApplication.class.getClassLoader());
        Reflections reflections = new Reflections("virtual_robot.controller.robots.classes");
        Set<Class<?>> configClasses = new HashSet<>();
        configClasses.addAll(reflections.getTypesAnnotatedWith(BotConfig.class));
        ObservableList<Class<?>> validConfigClasses = FXCollections.observableArrayList();
        for (Class<?> c: configClasses){
            if (!c.getAnnotation(BotConfig.class).disabled() && VirtualBot.class.isAssignableFrom(c))
                validConfigClasses.add(c);
        }
        cbxConfig.setItems(validConfigClasses);
        cbxConfig.setValue(MechanumBot.class);

        cbxConfig.setCellFactory(new Callback<ListView<Class<?>>, ListCell<Class<?>>>() {
            @Override
            public ListCell<Class<?>> call(ListView<Class<?>> param) {
                final ListCell<Class<?>> cell = new ListCell<Class<?>>(){
                    @Override
                    protected void updateItem(Class<?> cl, boolean bln){
                        super.updateItem(cl, bln);
                        if (cl == null){
                            setText(null);
                            return;
                        }
                        Annotation a = cl.getAnnotation(BotConfig.class);
                        setText(((BotConfig)a).name());
                    }
                };
                return cell;
            }
        });

        cbxConfig.setButtonCell(new ListCell<Class<?>>(){
            @Override
            protected void updateItem(Class<?> cl, boolean bln) {
                super.updateItem(cl, bln);
                if (cl == null) {
                    setText(null);
                    return;
                }
                Annotation a = cl.getAnnotation(BotConfig.class);
                setText(((BotConfig) a).name());
            }
        });
    }


    public VirtualBot getVirtualBotInstance(Class<?> c){
        try {
            Annotation a = c.getAnnotation(BotConfig.class);
            FXMLLoader loader = new FXMLLoader(getClass().getResource("/virtual_robot/controller/robots/fxml/" + ((BotConfig) a).filename() + ".fxml"));
            Group group = (Group) loader.load();
            VirtualBot bot = (VirtualBot) loader.getController();
            bot.setUpDisplayGroup(group);
            return bot;
        } catch (Exception e){
            System.out.println("Unable to load robot configuration.");
            System.out.println(e.getMessage());
            return null;
        }
    }

    private void setupCbxOpModes(){
        //Reflections reflections = new Reflections(VirtualRobotApplication.class.getClassLoader());
        Reflections reflections = new Reflections("org.firstinspires.ftc.teamcode");
        Set<Class<?>> opModes = new HashSet<>();
        opModes.addAll(reflections.getTypesAnnotatedWith(TeleOp.class));
        opModes.addAll(reflections.getTypesAnnotatedWith(Autonomous.class));
        nonDisabledOpModeClasses = FXCollections.observableArrayList();
        for (Class<?> c : opModes){
            if (c.getAnnotation(Disabled.class) == null && OpMode.class.isAssignableFrom(c)){
                nonDisabledOpModeClasses.add(c);
            }
        }

        nonDisabledOpModeClasses.sort(new Comparator<Class<?>>() {
            @Override
            public int compare(Class<?> o1, Class<?> o2) {
                String group1 = null;
                Annotation a1 = o1.getAnnotation(TeleOp.class);
                if (a1 != null) group1 = ((TeleOp)a1).group();
                else{
                    a1 = o1.getAnnotation(Autonomous.class);
                    if (a1 != null) group1 = ((Autonomous)a1).group();
                }

                String group2 = null;
                Annotation a2 = o2.getAnnotation(TeleOp.class);
                if (a2 != null) group2 = ((TeleOp)a2).group();
                else{
                    a2 = o2.getAnnotation(Autonomous.class);
                    if (a2 != null) group2 = ((Autonomous)a2).group();
                }

                if (group1 == null) return -1;
                else if (group2 == null) return 1;
                else return group1.compareToIgnoreCase(group2);
            }
        });

        cbxOpModes.setItems(nonDisabledOpModeClasses);

        cbxOpModes.setCellFactory(new Callback<ListView<Class<?>>, ListCell<Class<?>>>() {
            @Override
            public ListCell<Class<?>> call(ListView<Class<?>> param) {
                final ListCell<Class<?>> cell = new ListCell<Class<?>>(){
                    @Override
                    protected void updateItem(Class<?> cl, boolean bln){
                        super.updateItem(cl, bln);
                        if (cl == null){
                            setText(null);
                            return;
                        }
                        Annotation a = cl.getAnnotation(TeleOp.class);
                        if (a != null) setText(((TeleOp)a).group() + ": " + ((TeleOp)a).name());
                        else {
                            a = cl.getAnnotation(Autonomous.class);
                            if (a != null) setText(((Autonomous)a).group() + ": "  + ((Autonomous)a).name());
                            else setText("No Name");
                        }

                    }
                };
                return cell;
            }
        });

        cbxOpModes.setButtonCell(new ListCell<Class<?>>(){
            @Override
            protected void updateItem(Class<?> cl, boolean bln) {
                super.updateItem(cl, bln);
                if (cl == null) {
                    setText(null);
                    return;
                }
                Annotation a = cl.getAnnotation(TeleOp.class);
                if (a != null) setText(((TeleOp) a).name());
                else {
                    a = cl.getAnnotation(Autonomous.class);
                    if (a != null) setText(((Autonomous) a).name());
                    else setText("No Name");
                }
            }
        });

        cbxOpModes.setValue(cbxOpModes.getItems().get(0));
    }


    @FXML
    public void setConfig(ActionEvent event){
        if (opModeInitialized || opModeStarted) return;
        if (bot != null) bot.removeFromDisplay(fieldPane);
        bot = getVirtualBotInstance(cbxConfig.getValue());
        if (bot == null) System.out.println("Unable to get VirtualBot Object");
        hardwareMap = bot.getHardwareMap();
        initializeTelemetryTextArea();
        sldRandomMotorError.setValue(0.0);
        sldSystematicMotorError.setValue(0.0);
        sldMotorInertia.setValue(0.0);
    }

    public StackPane getFieldPane(){ return fieldPane; }

    @FXML
    private void handleDriverButtonAction(ActionEvent event){
        if (!opModeInitialized){
            if (!initOpMode()) return;
            pathLine.getPoints().clear();
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
                    pathLine.getPoints().addAll(halfFieldWidth + bot.x, halfFieldWidth - bot.y);
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
            //if (opModeThread.isAlive() && !opModeThread.isInterrupted()) opModeThread.interrupt();
            if (!executorService.isShutdown()) executorService.shutdown();
            try{
                opModeThread.join(500);
            } catch(InterruptedException exc) {
                Thread.currentThread().interrupt();
            }
            if (opModeThread.isAlive()) System.out.println("OpMode Thread Failed to Terminate.");
            bot.powerDownAndReset();
            if (Config.USE_VIRTUAL_GAMEPAD) virtualGamePadController.resetGamePad();
            initializeTelemetryTextArea();
            cbxConfig.setDisable(false);
        }
    }

    private void runOpModeAndCleanUp(){

        try {
            //For regular opMode, run user-defined init() method. For Linear opMode, init() starts the execution of
            //runOpMode on a helper thread.
            opMode.init();

            while (!opModeStarted && !Thread.currentThread().isInterrupted()) {
                //For regular opMode, run user-defined init_loop() method. For Linear opMode, init_loop checks whether
                //runOpMode has exited; if so, it interrupts the opModeThread.
                opMode.init_loop();
                //For regular op mode, update telemetry after each iteration of init_loop()
                //For linear op mode, do-nothing
                opMode.internalPostInitLoop();

                try {
                    Thread.sleep(0, 1);
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                }

            }

            //For regular opMode, run user-defined stop() method, if any. For Linear opMode, the start() method
            //will allow waitForStart() to finish executing.
            if (!Thread.currentThread().isInterrupted()) opMode.start();

            while (opModeStarted && !Thread.currentThread().isInterrupted()) {
                //For regular opMode, run user-defined loop() method. For Linear opMode, loop() checks whether
                //runOpMode has exited; if so, it interrupts the opModeThread.
                opMode.loop();
                //For regular op mode only, update telemetry after each execution of loop()
                //For linear op mode, do-nothing
                opMode.internalPostLoop();

                try {
                    Thread.sleep(0, 1);
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                }

            }

            //For regular opMode, run user-defined stop() method, if any. For Linear opMode, shut down the
            //helper thread that runs runOpMode.
            opMode.stop();
        } catch(Exception e){
            System.out.println("Exception thrown by opModeThread.");
            System.out.println(e.getClass().getName());
            System.out.println(e.getLocalizedMessage());
        }

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
                if (Config.USE_VIRTUAL_GAMEPAD) virtualGamePadController.resetGamePad();
            }
        });

        System.out.println("Finished executing runOpModeAndCleanUp() on opModeThread.");
    }

    @FXML
    private void handleFieldMouseClick(MouseEvent arg){
        if (opModeInitialized || opModeStarted) return;
        bot.positionWithMouseClick(arg);
    }


    private boolean initOpMode() {
        try {
            Class opModeClass = cbxOpModes.getValue();
            opMode = (OpMode) opModeClass.newInstance();
        } catch (Exception exc){
            return false;
        }
        return true;
    }

    @FXML
    private void handleCbxShowPathAction(ActionEvent event){
        if (pathLine == null) return;
        pathLine.setVisible(cbxShowPath.isSelected());
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
        Set<String> crservos = hardwareMap.crservo.keySet();
        if (!crservos.isEmpty()){
            sb.append("\n CR Servos:");
            for (String crservo : crservos) sb.append("\n   " + crservo);
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

        public synchronized void updateColor(double x, double y){
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


    /**
     * Base class for OpMode.
     */
    public class OpModeBase {
        protected final HardwareMap hardwareMap;
        protected final GamePad gamepad1;
        protected final GamePad gamepad2;
        protected final Telemetry telemetry;

        public OpModeBase() {
            hardwareMap = VirtualRobotController.this.hardwareMap;
            gamepad1 = gamePad1;
            this.gamepad2 = gamePad2;
            telemetry = new TelemetryImpl();
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

    public interface GamePadHelper extends Runnable{
        public void quit();
    }

    public class VirtualGamePadHelper implements GamePadHelper {

        public void run() {
            VirtualGamePadController.ControllerState state = virtualGamePadController.getState();
            gamePad1.update(state);
            gamePad2.resetValues();
        }

        public void quit(){}
    }

    public class RealGamePadHelper implements  GamePadHelper {

        private int gamePad1Index = -1;
        private int gamePad2Index = -1;

        private boolean isConnected0 = true;
        private boolean isConnected1 = true;

        private boolean changingGamePadConfig = false;

        private ControllerManager controller = null;

        public RealGamePadHelper(){
            controller = new ControllerManager(2);
            controller.initSDLGamepad();
        }

        public void run(){
            boolean connectionChanged = false;
            boolean configChanged = false;

            ControllerState state0 = controller.getState(0);
            ControllerState state1 = controller.getState(1);

            if (state0.isConnected != isConnected0 || state1.isConnected != isConnected1){
                isConnected0 = state0.isConnected;
                isConnected1 = state1.isConnected;
                connectionChanged = true;
                System.out.println("isConnected0 = " + isConnected0 + "  isConnected1 = " + isConnected1);
            }

            if (state0.start && (state0.a || state0.b) || state1.start && (state1.a || state1.b)) {
                if (!changingGamePadConfig) {

                    changingGamePadConfig = true;

                    if (state0.start && state0.a) {
                        gamePad1Index = 0;
                        if (gamePad2Index == 0) gamePad2Index = -1;
                    } else if (state0.start && state0.b) {
                        gamePad2Index = 0;
                        if (gamePad1Index == 0) gamePad1Index = -1;
                    }

                    if (state1.start && state1.a) {
                        gamePad1Index = 1;
                        if (gamePad2Index == 1) gamePad2Index = -1;
                    } else if (state1.start && state1.b) {
                        gamePad2Index = 1;
                        if (gamePad1Index == 1) gamePad1Index = -1;
                    }

                    System.out.println("gamepad1 index = " + gamePad1Index + "   gamepad2 index = " + gamePad2Index);

                    configChanged = true;
                }
            } else {
                changingGamePadConfig = false;
            }

            if (configChanged || connectionChanged){
                Platform.runLater(new Runnable() {
                    @Override
                    public void run() {
                        checkBoxGamePad1.setSelected(gamePad1Index == 0 && isConnected0 || gamePad1Index == 1 && isConnected1);
                        checkBoxGamePad2.setSelected(gamePad2Index == 0 && isConnected0 || gamePad2Index == 1 && isConnected1);
                    }
                });
            }


            if (gamePad1Index == 0) gamePad1.update(state0);
            else if (gamePad1Index == 1) gamePad1.update(state1);
            else gamePad1.resetValues();

            if (gamePad2Index == 0) gamePad2.update(state0);
            else if (gamePad2Index == 1) gamePad2.update(state1);
            else gamePad2.resetValues();
        }

        public void quit(){
            controller.quitSDLGamepad();
        }

    }

}
