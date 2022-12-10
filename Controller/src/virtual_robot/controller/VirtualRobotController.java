package virtual_robot.controller;

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.Range;
import com.studiohartman.jamepad.ControllerIndex;
import com.studiohartman.jamepad.ControllerManager;
import com.studiohartman.jamepad.ControllerState;
import com.studiohartman.jamepad.ControllerUnpluggedException;
import javafx.beans.value.ChangeListener;
import javafx.beans.value.ObservableValue;
import javafx.collections.FXCollections;
import javafx.collections.ObservableList;
import javafx.fxml.FXMLLoader;
import javafx.scene.Group;
import javafx.scene.control.*;
import javafx.scene.input.KeyCode;
import javafx.scene.input.KeyEvent;
import javafx.scene.layout.BorderPane;
import javafx.scene.layout.HBox;
import javafx.scene.layout.Pane;
import javafx.scene.shape.Polyline;
import javafx.scene.shape.Rectangle;
import javafx.util.Callback;
import org.dyn4j.dynamics.Body;
import org.dyn4j.dynamics.BodyFixture;
import org.dyn4j.dynamics.ContinuousDetectionMode;
import org.dyn4j.geometry.MassType;
import org.dyn4j.world.World;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.opmode.TelemetryImpl;
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
import javafx.scene.paint.Color;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotorImpl;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import virtual_robot.robots.ControlsElements;
import virtual_robot.robots.classes.MecanumBot;
import virtual_robot.keyboard.KeyState;

import java.io.IOException;
import java.lang.annotation.Annotation;
import java.util.*;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

/**
 * For internal use only. Controller class for the JavaFX application.
 */
public class VirtualRobotController {

    //User Interface
    @FXML private Pane fieldPane;
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
    @FXML private CheckBox checkBoxAutoHuman;

    // dyn4j world
    World<Body> world = new World<>();

    // Virtual Hardware
    private HardwareMap hardwareMap = null;
    private VirtualBot bot = null;
    Gamepad gamePad1 = new Gamepad();
    Gamepad gamePad2 = new Gamepad();
    GamePadHelper gamePadHelper = null;
    ScheduledExecutorService gamePadExecutorService = Executors.newSingleThreadScheduledExecutor();

    VirtualGamePadController virtualGamePadController = null;

    //Background Image and Field
    private final Image backgroundImage = Config.BACKGROUND;
    private final PixelReader pixelReader = backgroundImage.getPixelReader();
    private double halfFieldWidth;
    private double fieldWidth;

    //Path Drawing
    Polyline pathLine;



    //OpMode Control
    private OpMode opMode = null;
    private volatile boolean opModeInitialized = false;
    private volatile boolean opModeStarted = false;
    private Thread opModeThread = null;

    //Virtual Robot Control Engine
    ScheduledExecutorService executorService = null;
    public static final double TIME_INTERVAL_MILLISECONDS = 20;


    //Random Number Generator
    private final Random random = new Random();

    //KeyState
    private final KeyState keyState = new KeyState();

    /*
     * Motor slider listener
     *
     * The values set for random error fraction and systematic error fraction may look reversed, but they aren't.
     * Systematic error fraction is set randomly for each motor when the slider is changed, but then remains the
     * same for that motor until the slider is changed again. Random error fraction is set for each motor when the
     * slider is changed, but then (in the DcMotorImpl class) gets multiplied by a new random number during each motor
     * update cycle.
     */
    private final ChangeListener<Number> sliderChangeListener = new ChangeListener<Number>() {
        @Override
        public void changed(ObservableValue<? extends Number> observable, Number oldValue, Number newValue) {
            for (DcMotor motor: hardwareMap.dcMotor) {
                if (!(motor instanceof DcMotorImpl)) continue;      //Now that DeadWheelEncoder has been added, not all "DcMotor" are "DcMotorImpl"
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
        VirtualGameElement.setController(this);
        Game.setController(this);
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

        addConstraintMasks();

        setupPhysicsWorld();

        Config.GAME.initialize();
        Config.GAME.resetGameElements();
        Config.GAME.setHumanPlayerAuto(true);

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

    /**
     *  Adjust world settings (especially gravity, but other settings may need adjustment as well).
     *  Add Bodys (with rectangular BodyFixtures) on all four sides, representing the walls.
     */
    private void setupPhysicsWorld(){
        world.setGravity(0, 0);
        world.getSettings().setContinuousDetectionMode(ContinuousDetectionMode.BULLETS_ONLY);
        world.getSettings().setBaumgarte(0.2);

        // Create Rectangles for four 1 meter thick walls
        org.dyn4j.geometry.Rectangle topRect = new org.dyn4j.geometry.Rectangle(
                VirtualField.FIELD_WIDTH_METERS + 2, 1);
        org.dyn4j.geometry.Rectangle bottomRect = new org.dyn4j.geometry.Rectangle(
                VirtualField.FIELD_WIDTH_METERS + 2, 1);
        org.dyn4j.geometry.Rectangle leftRect = new org.dyn4j.geometry.Rectangle(
                1, VirtualField.FIELD_WIDTH_METERS);
        org.dyn4j.geometry.Rectangle rightRect = new org.dyn4j.geometry.Rectangle(
                1, VirtualField.FIELD_WIDTH_METERS);

        // Translate the rectangles into correct positions
        topRect.translate(0, VirtualField.Y_MAX/VirtualField.PIXELS_PER_METER + 0.5);
        bottomRect.translate(0, VirtualField.Y_MIN/VirtualField.PIXELS_PER_METER - 0.5);
        leftRect.translate(VirtualField.X_MIN/VirtualField.PIXELS_PER_METER - 0.5, 0);
        rightRect.translate(VirtualField.X_MAX/VirtualField.PIXELS_PER_METER + 0.5, 0);

        /*
         * For each wall, create a body with infinite mass. The shape (i.e., Rectangle) for each wall is placed into
         * the body via a BodyFixture. The Fixture is assigned a Category filter which assigns it to the WALL
         * category, and allows it to collide with all categories.
         */
        Body topWall = new Body();
        BodyFixture topFixture = topWall.addFixture(topRect);
        topFixture.setFilter(Filters.WALL_FILTER);
        topWall.setMass(MassType.INFINITE);
        world.addBody(topWall);
        topWall.setUserData(new Wall());

        Body bottomWall = new Body();
        BodyFixture bottomFixture = bottomWall.addFixture(bottomRect);
        bottomFixture.setFilter(Filters.WALL_FILTER);
        bottomWall.setMass(MassType.INFINITE);
        world.addBody(bottomWall);
        bottomWall.setUserData(new Wall());

        Body leftWall = new Body();
        BodyFixture leftFixture = leftWall.addFixture(leftRect);
        leftFixture.setFilter(Filters.WALL_FILTER);
        leftWall.setMass(MassType.INFINITE);
        world.addBody(leftWall);
        leftWall.setUserData(new Wall());

        Body rightWall = new Body();
        BodyFixture rightFixture = rightWall.addFixture(rightRect);
        rightFixture.setFilter(Filters.WALL_FILTER);
        rightWall.setMass(MassType.INFINITE);
        world.addBody(rightWall);
        rightWall.setUserData(new Wall());
    }

    /**
     *  "Gray out" part of the field based on the field constraints (X_MIN_FRACTION, X_MAX_FRACTION,
     *  Y_MIN_FRACTION, Y_MAX_FRACTION values)
     */
    private void addConstraintMasks(){
        if (Config.X_MIN_FRACTION > 0){
            Rectangle rect = new Rectangle(fieldWidth*Config.X_MIN_FRACTION, fieldWidth);
            rect.setFill(Color.color(0.2, 0.2, 0.2, 0.75));
            fieldPane.getChildren().add(rect);
        }
        if (Config.X_MAX_FRACTION < 1){
            Rectangle rect = new Rectangle(fieldWidth*(1-Config.X_MAX_FRACTION), fieldWidth);
            rect.setTranslateX(fieldWidth*Config.X_MAX_FRACTION);
            rect.setFill(Color.color(0.2, 0.2, 0.2, 0.75));
            fieldPane.getChildren().add(rect);
        }
        if (Config.Y_MIN_FRACTION > 0){
            Rectangle rect = new Rectangle(fieldWidth*(Config.X_MAX_FRACTION-Config.X_MIN_FRACTION), fieldWidth*Config.Y_MIN_FRACTION);
            rect.setTranslateX(fieldWidth*Config.X_MIN_FRACTION);
            rect.setTranslateY(fieldWidth*(1-Config.Y_MIN_FRACTION));
            rect.setFill(Color.color(0.2, 0.2, 0.2, 0.75));
            fieldPane.getChildren().add(rect);
        }
        if (Config.Y_MAX_FRACTION < 1){
            Rectangle rect = new Rectangle(fieldWidth*(Config.X_MAX_FRACTION-Config.X_MIN_FRACTION), fieldWidth*(1-Config.Y_MAX_FRACTION));
            rect.setTranslateX(fieldWidth*Config.X_MIN_FRACTION);
            rect.setFill(Color.color(0.2, 0.2, 0.2, 0.75));
            fieldPane.getChildren().add(rect);
        }
    }

    private void setupCbxRobotConfigs(){
        //Reflections reflections = new Reflections(VirtualRobotApplication.class.getClassLoader());
        Reflections reflections = new Reflections("virtual_robot.robots.classes");
        Set<Class<?>> configClasses = new HashSet<>();
        configClasses.addAll(reflections.getTypesAnnotatedWith(BotConfig.class));
        ObservableList<Class<?>> validConfigClasses = FXCollections.observableArrayList();
        for (Class<?> c: configClasses){
            if (!c.getAnnotation(BotConfig.class).disabled() && VirtualBot.class.isAssignableFrom(c))
                validConfigClasses.add(c);
        }
        cbxConfig.setItems(validConfigClasses);
        cbxConfig.setValue(MecanumBot.class);

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
            FXMLLoader loader = new FXMLLoader(getClass().getResource("/virtual_robot/robots/fxml/" + ((BotConfig) a).filename() + ".fxml"));
            Group group = (Group) loader.load();
            VirtualBot bot = (VirtualBot) loader.getController();
            bot.setUpDisplayGroup(group);
            return bot;
        } catch (Exception e){
            System.out.println("Unable to load robot configuration.");
            System.out.println(e.getMessage());
            e.printStackTrace();
            return null;
        }
    }


    private String getNameFromAnnotationOrOpmode(Class c){
        String name = "";
        Annotation a1 = c.getAnnotation(TeleOp.class);
        if(a1 != null){
            name = ((TeleOp)a1).name();
        }else{
            a1 = c.getAnnotation(Autonomous.class);
            if(a1 != null){
                name = ((Autonomous)a1).name();
            }
        }
        if(name.isEmpty()){
            name = c.getSimpleName();
        }
        return name;
    }

    private String getGroupFromAnnotationOrOpmode(Class c){
        String group = null;
        Annotation a1 = c.getAnnotation(TeleOp.class);
        if(a1 != null){
            group = ((TeleOp)a1).group();
        }else{
            a1 = c.getAnnotation(Autonomous.class);
            if(a1 != null){
                group = ((Autonomous)a1).group();
            }
        }
        return group;
    }

    private void setupCbxOpModes(){
//        Reflections reflections = new Reflections("");
        Reflections reflections = new Reflections("org.firstinspires.ftc.teamcode");
        Set<Class<?>> opModes = new HashSet<>();
        opModes.addAll(reflections.getTypesAnnotatedWith(TeleOp.class));
        opModes.addAll(reflections.getTypesAnnotatedWith(Autonomous.class));//Lists of OpMode classes and OpMode Names
        ObservableList<Class<?>> nonDisabledOpModeClasses = FXCollections.observableArrayList();
        for (Class<?> c : opModes){
            if (c.getAnnotation(Disabled.class) == null && OpMode.class.isAssignableFrom(c)){
                nonDisabledOpModeClasses.add(c);
            }
        }

        nonDisabledOpModeClasses.sort(new Comparator<Class<?>>() {
            @Override
            public int compare(Class<?> o1, Class<?> o2) {
                String group1 = getGroupFromAnnotationOrOpmode(o1);
                String group2 = getGroupFromAnnotationOrOpmode(o2);

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
                        String group = getGroupFromAnnotationOrOpmode(cl);
                        String name = getNameFromAnnotationOrOpmode(cl);

                        if(group.isEmpty()) {
                            setText(name);
                        }else{
                            setText(group + ": " + name);
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
                setText(getNameFromAnnotationOrOpmode(cl));
            }
        });

        cbxOpModes.setValue(cbxOpModes.getItems().get(0));
    }


    @FXML
    public void setConfig(ActionEvent event){
        if (opModeInitialized || opModeStarted) return;
        if (bot != null) {
            bot.removeFromWorld();
            bot.removeFromDisplay(fieldPane);
        }
        bot = getVirtualBotInstance(cbxConfig.getValue());
        if (bot == null) System.out.println("Unable to get VirtualBot Object");
        hardwareMap = bot.getHardwareMap();
        initializeTelemetryTextArea();
        sldRandomMotorError.setValue(0.0);
        sldSystematicMotorError.setValue(0.0);
        sldMotorInertia.setValue(0.0);
    }


    public Pane getFieldPane(){ return fieldPane; }

    public World<Body> getWorld(){ return world; }

    @FXML
    private void handleDriverButtonAction(ActionEvent event){
        if (!opModeInitialized){
            /*
             * INIT has been pressed.
             */
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
                    Config.GAME.updateDisplay();
                    bot.updateDisplay();
                    pathLine.getPoints().addAll(halfFieldWidth + bot.getX(), halfFieldWidth - bot.getY());
                }
            };

            Runnable singleCycle = new Runnable() {
                @Override
                public void run() {
                    singlePhysicsCycle();
                    Platform.runLater(updateDisplay);
                }
            };
            executorService = Executors.newSingleThreadScheduledExecutor();
            executorService.scheduleAtFixedRate(singleCycle, 0, (long) TIME_INTERVAL_MILLISECONDS, TimeUnit.MILLISECONDS);

            opModeThread.start();
        } else if (!opModeStarted){
            /*
             * START has been pressed.
             */
            driverButton.setText("STOP");
            opModeStarted = true;
        } else{
            /*
             * STOP has been pressed. Note that it is not possible for this to happen before START is pressed.
             */
            driverButton.setText("INIT");
            opModeInitialized = false;
            /*
             * Setting opModeStarted to false will:
             *   -Cause the final loop in runOpModeAndCleanUp to exit;
             *   -Cause opmode.Stop() to run
             *   -In a linear opmode, the above will cause stopRequested to become true, and interrupt runOpMode thread
             */
            opModeStarted = false;
            if (!executorService.isShutdown()) executorService.shutdown();
            /*
             * This should not be necessary, but...
             */
            try{
                opModeThread.join(500);
            } catch(InterruptedException exc) {
                opModeThread.interrupt();
            }
            if (opModeThread.isAlive()) System.out.println("OpMode Thread Failed to Terminate.");

            bot.getHardwareMap().setActive(false);
            bot.powerDownAndReset();
            Config.GAME.stopGameElements();
            gamePadHelper.onOpModeFinished();
            initializeTelemetryTextArea();
            cbxConfig.setDisable(false);
        }
    }

    private void runOpModeAndCleanUp(){

        try {
            //Activate the hardware map, so that calls to "get" on the hardware map itself, and on dcMotor, etc,
            //will return hardware objects
            bot.getHardwareMap().setActive(true);

            //For regular opMode, run user-defined init() method. For Linear opMode, init() starts the execution of
            //runOpMode on a helper thread.
            opMode.init();

            while (!opModeStarted && !Thread.currentThread().isInterrupted()) {
                // to keep the guarantee that this is updated
                opMode.time = opMode.getRuntime();
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

            //For regular opMode, run user-defined start() method, if any. For Linear opMode, the start() method
            //will allow waitForStart() to finish executing.
            if (!Thread.currentThread().isInterrupted()) opMode.start();

            while (opModeStarted && !Thread.currentThread().isInterrupted()) {
                //For regular opMode, run user-defined loop() method. For Linear opMode, loop() checks whether
                //runOpMode has exited; if so, it interrupts the opModeThread.

                // to keep the guarantee that this is updated
                opMode.time = opMode.getRuntime();

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
            System.out.println("Stack Trace:");
            for (StackTraceElement stackTraceElement: e.getStackTrace()){
                System.out.println("  " + stackTraceElement.toString());
            }
            System.out.println();
        }

        bot.getHardwareMap().setActive(false);
        bot.powerDownAndReset();
        Config.GAME.stopGameElements();
        if (!executorService.isShutdown()) executorService.shutdown();
        opModeInitialized = false;
        opModeStarted = false;
        gamePadHelper.onOpModeFinished();
        Platform.runLater(new Runnable() {
            public void run() {
                driverButton.setText("INIT");
                //resetGamePad();
                initializeTelemetryTextArea();
                cbxConfig.setDisable(false);
            }
        });

        System.out.println("Finished executing runOpModeAndCleanUp() on opModeThread.");
    }

    private void singlePhysicsCycle(){
        // Update the physics engine. This will also call any collision/contact listeners that have been set.
        // These listeners will generally be in the bot class. They should record events within fields in the bot's
        // class, to be handled later in the bot.updateStateAndSensors call.
        world.updatev(TIME_INTERVAL_MILLISECONDS / 1000.0);

        // Update game element pose, and any other relevant state, of all game elements
        Config.GAME.updateGameElementState(TIME_INTERVAL_MILLISECONDS);

        // Update robot's pose by obtaining it from the physics engine, accumulate forces on Body based on
        // drive motor status, and update robot sensors. Depending on collision/contact events that occurred
        // during the world.updatev() call, it is also possible that this method will directly affect game elements.
        bot.updateStateAndSensors(TIME_INTERVAL_MILLISECONDS);


        if (Config.GAME.hasHumanPlayer() && Config.GAME.isHumanPlayerAuto() && opModeStarted
                || Config.GAME.isHumanPlayerActionRequested() && opModeInitialized) {
            Config.GAME.updateHumanPlayerState(TIME_INTERVAL_MILLISECONDS);
        }
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

    public void updateTelemetryDisplay(String telemetryText) {
        txtTelemetry.setText(telemetryText);
    }
		
    @FXML
    private void handleCheckBoxAutoHumanAction(ActionEvent event){
        Config.GAME.setHumanPlayerAuto(checkBoxAutoHuman.isSelected());
    }

    @FXML
    private void handleBtnHumanAction(ActionEvent event){
        if (opModeInitialized) Config.GAME.requestHumanPlayerAction();
    }

    @FXML
    private void handleBtnResetGameElements(ActionEvent event){
        if (opModeInitialized) return;
        if (bot instanceof ControlsElements) ((ControlsElements) bot).clearLoadedElements(Config.GAME);
        Config.GAME.resetGameElements();
    }

    @FXML
    private void handleBtnPreloadElementsOnBot(ActionEvent event){
        if (!opModeInitialized && bot instanceof ControlsElements){
            ((ControlsElements) bot).preloadElements(Config.GAME);
        }
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
        Set<String> digitalChannels = hardwareMap.keySet(DigitalChannel.class);
        if (!digitalChannels.isEmpty()) {
            sb.append("\n Digital Sensors:");
            for (String digitalChannel : digitalChannels) sb.append("\n   " + digitalChannel);
        }
        Set<String> analogInputs = hardwareMap.keySet(AnalogInput.class);
        if (!analogInputs.isEmpty()) {
            sb.append("\n Analog Sensors:");
            for (String analogInput : analogInputs) sb.append("\n   " + analogInput);
        }
        txtTelemetry.setText(sb.toString());
    }

    @FXML
    private void handleKeyEvents(KeyEvent e){
        if (e.getEventType() == KeyEvent.KEY_PRESSED){
            keyState.set(e.getCode(), true);
        } else if (e.getEventType() == KeyEvent.KEY_RELEASED){
            keyState.set(e.getCode(), false);
        }
    }

    public boolean getKeyState(KeyCode code){
        return keyState.get(code);
    }

    public class ColorSensorImpl implements ColorSensor, ColorRangeSensor {
        private int red = 0;
        private int green = 0;
        private int blue = 0;
        private int alpha = 0;
        private double gain = 1.0;
        private double distanceCM = 0.0;

        public synchronized int red() {
            return red;
        }

        public synchronized int green() {
            return green;
        }

        public synchronized int blue() {
            return blue;
        }

        public synchronized int alpha() {
            return alpha;
        }

        public synchronized void updateColor(double x, double y) {
            int colorX = (int) (x + halfFieldWidth);
            int colorY = (int) (halfFieldWidth - y);
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
            tempGreen = Math.floor(tempGreen * 256.0 / 81.0);
            if (tempGreen == 256) tempGreen = 255;
            tempBlue = Math.floor(tempBlue * 256.0 / 81.0);
            if (tempBlue == 256) tempBlue = 255;
            red = (int) tempRed;
            green = (int) tempGreen;
            blue = (int) tempBlue;
            alpha = Math.max(red, Math.max(green, blue));
        }

        public synchronized void setDistance(double distance, DistanceUnit distanceUnit) {
            distanceCM = distanceUnit.toCm(distance);
        }

        @Override
        public synchronized double getDistance(DistanceUnit distanceUnit) {
            return distanceUnit.fromCm(distanceCM);
        }

        @Override
        public double getLightDetected() {
            return 0;
        }

        @Override
        public double getRawLightDetected() {
            return 0;
        }

        @Override
        public double getRawLightDetectedMax() {
            return 1.0;
        }

        @Override
        public void enableLed(boolean enable) {

        }

        @Override
        public String status() {
            return String.format(Locale.getDefault(), "%s on %s", getDeviceName(), getConnectionInfo());
        }

        @Override
        public synchronized NormalizedRGBA getNormalizedColors() {
            NormalizedRGBA nRGBA = new NormalizedRGBA();
            nRGBA.red = red / 256.0f;
            nRGBA.green = green / 256.0f;
            nRGBA.blue = blue / 256.0f;
            nRGBA.alpha = alpha / 256.0f;
            return nRGBA;
        }

        @Override
        public synchronized float getGain() {
            return (float) gain;
        }

        @Override
        public synchronized void setGain(float newGain) {
            gain = newGain;
        }
    }

    public class DistanceSensorImpl implements DistanceSensor {

        private final double readingWhenOutOfRangeMM = 8200;
        private double distanceMM = readingWhenOutOfRangeMM;
        private static final double MIN_DISTANCE = 50; //mm
        private static final double MAX_DISTANCE = 1000; //mm
        private static final double MAX_OFFSET = 7.0 * Math.PI / 180.0;

        private final double X_MIN, X_MAX, Y_MIN, Y_MAX;    //Need these to constrain field

        public DistanceSensorImpl(){
            X_MIN = 2.0 * (Config.X_MIN_FRACTION - 0.5) * halfFieldWidth;
            X_MAX = 2.0 * (Config.X_MAX_FRACTION - 0.5) * halfFieldWidth;
            Y_MIN = 2.0 * (Config.Y_MIN_FRACTION - 0.5) * halfFieldWidth;
            Y_MAX = 2.0 * (Config.Y_MAX_FRACTION - 0.5) * halfFieldWidth;
        }

        public synchronized double getDistance(DistanceUnit distanceUnit){
            double result;
            if (distanceMM < MIN_DISTANCE) result = MIN_DISTANCE - 1.0;
            else if (distanceMM > MAX_DISTANCE) result = readingWhenOutOfRangeMM;
            else result = distanceMM;
            return distanceUnit.fromMm(result);
        }

        public synchronized void updateDistance(double x, double y, double headingRadians){
            final double mmPerPixel = 144.0 * 25.4 / fieldWidth;
            final double piOver2 = Math.PI / 2.0;
            double temp = headingRadians / piOver2;
            int side = (int)Math.round(temp); //-2, -1 ,0, 1, or 2 (2 and -2 both refer to the bottom)
            double offset = Math.abs(headingRadians - (side * Math.PI / 2.0));
            if (offset > MAX_OFFSET) distanceMM = readingWhenOutOfRangeMM;
            else switch (side){
                case 2:
                case -2:
                    distanceMM = (y - Y_MIN) * mmPerPixel;                  //BOTTOM
                    break;
                case -1:
                    distanceMM = (X_MAX - x) * mmPerPixel;         //RIGHT
                    break;
                case 0:
                    distanceMM = (Y_MAX - y) * mmPerPixel;         //TOP
                    break;
                case 1:
                    distanceMM = (x - X_MIN) * mmPerPixel;         //LEFT
                    break;
            }
        }

    }


    /**
     * Base class for OpMode.
     */
    public class OpModeBase {
        public final HardwareMap hardwareMap;
        public final Gamepad gamepad1;
        public final Gamepad gamepad2;
        public final Telemetry telemetry;

        public OpModeBase() {
            hardwareMap = VirtualRobotController.this.hardwareMap;
            gamepad1 = gamePad1;
            this.gamepad2 = gamePad2;
            telemetry = new TelemetryImpl(VirtualRobotController.this);
        }
    }


    public interface GamePadHelper extends Runnable{
        public void quit();
        public void onOpModeFinished();
    }

    public class VirtualGamePadHelper implements GamePadHelper {

        public void run() {
            VirtualGamePadController.ControllerState state = virtualGamePadController.getState();
            gamePad1.update(state);
            virtualGamePadController.setOutputs(gamePad1);
            gamePad2.resetValues();
        }

        public void quit(){
            //Make sure that LED and Rumble threads are interrupted if user closes the application while an op mode is
            //running.
            virtualGamePadController.interruptLEDandRumbleThreads();
        }

        public void onOpModeFinished(){
            virtualGamePadController.resetGamePad();
        }
    }

    public class RealGamePadHelper implements  GamePadHelper {

        private int gamePad1Index = -1;
        private int gamePad2Index = -1;

        private boolean isConnected0 = true;
        private boolean isConnected1 = true;

        private boolean changingGamePadConfig = false;

        private ControllerManager controller = null;

        private Thread[] rumbleThreads = new Thread[2];

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

            setOutputs(gamePad1, gamePad1Index);
            setOutputs(gamePad2, gamePad2Index);
        }

        public void setOutputs(Gamepad gamepad, final int gamePadIndex){
            if (gamePadIndex <0 || gamePadIndex >1) return;
            ControllerIndex controllerIndex = controller.getControllerIndex(gamePadIndex);
            Gamepad.RumbleEffect rumbles = gamepad.rumbleQueue.poll();
            if (rumbles == null) return;
            if (rumbleThreads[gamePadIndex] != null){
                rumbleThreads[gamePadIndex].interrupt();
            }

            // Make this final so accessable from rumble thread
            final ListIterator<Gamepad.RumbleEffect.Step> stepIterator = rumbles.steps.listIterator();

            rumbleThreads[gamePadIndex] = new Thread(new Runnable() {
                @Override
                public void run() {
                    while (stepIterator.hasNext()) {
                        Gamepad.RumbleEffect.Step step = stepIterator.next();
                        float leftMagnitude = (float)Range.scale((double)step.large, 0, 255, 0, 1);
                        float rightMagnitude = (float)Range.scale((double)step.small, 0, 255, 0, 1);

                        try {
                            controllerIndex.doVibration(leftMagnitude, rightMagnitude, 300000);
                        } catch (ControllerUnpluggedException ex) {
                            return;
                        }
                        if (step.duration == -1) {
                            return;
                        }
                        try {
                            Thread.sleep(step.duration);
                        } catch (InterruptedException e) {
                            return;  // don't know why it was interrupted, but lets just bail on this sequence
                        }
                    }

                    try{
                        controllerIndex.doVibration(0, 0, 300000);
                    } catch (ControllerUnpluggedException ex) {
                        return;
                    }

                }
            });

            rumbleThreads[gamePadIndex].start();
        }


        public void quit(){
            interruptRumbleThreads();
            controller.quitSDLGamepad();
        }

        public void onOpModeFinished(){
            interruptRumbleThreads();
        }

        public void interruptRumbleThreads(){
            if (rumbleThreads[0] != null){
                rumbleThreads[0].interrupt();
            }
            if (rumbleThreads[1] != null){
                rumbleThreads[1].interrupt();
            }
        }

    }

}
