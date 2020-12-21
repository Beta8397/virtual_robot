package system.robot;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import sim_config.SimConfig;
import system.config.*;
import system.gui.HALGUI;
import system.gui.Payload;
import system.gui.menus.configmenu.ConfigConstants;
import system.gui.menus.configmenu.ConfigStartingMenu;
import util.control.Button;
import util.control.CustomizableGamepad;
import util.exceptions.DumpsterFireException;
import util.exceptions.ExceptionChecker;
import util.exceptions.NothingToSeeHereException;
import util.misc.HALFileUtil;

import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Queue;
import java.util.concurrent.LinkedBlockingQueue;

import static system.config.ConfigSelectionMode.AUTONOMOUS;
import static system.config.ConfigSelectionMode.TELEOP;

/**
 * An abstract class representing the physical robot. This class is the central hub for most of HAL.
 * <p>
 * Creation Date: 2017
 *
 * @author Andrew Liang, Level Up
 * @author Dylan Zueck, Crow Force
 * @author Cole Savage, Level Up
 * @version 1.0.0
 * @see SubSystem
 * @see HALProgram
 * @see HALGUI
 * @see HALConfig
 * @since 0.0.0
 */
@SuppressWarnings({"WeakerAccess"})
public abstract class Robot {
    //The path to HAL's root config folder.
    private static final String HAL_FILESYSTEM_ROOT = SimConfig.HAL_FILESYSTEM_ROOT + "/System64";
    //A queue of all added subsystems.
    private final Queue<SubSystem> subSystems;
    //The global HAL config.
    private HALConfig globalConfig;
    //The opmode the robot is running.
    private OpMode opMode;
    //The GUI the robot uses to render the menus.
    public HALGUI gui;
    //A boolean value specifying whether or not to use the config system.
    private boolean useConfig = false;
    //The gamepads used to control the robot.
    public volatile Gamepad gamepad1, gamepad2;
    //The telemetry used to print lines to the driver station.
    public final Telemetry telemetry;
    //The hardwaremap used to map software representations of hardware to the actual hardware.
    public final HardwareMap hardwareMap;

    /**
     * Constructor for robot.
     *
     * @param opMode The opmode the robot is currently running.
     *
     * @see HALProgram
     * @see HALGUI
     */
    public Robot(OpMode opMode) {
        this.opMode = opMode;
        telemetry = opMode.telemetry;
        hardwareMap = opMode.hardwareMap;

        subSystems = new LinkedBlockingQueue<>();

        globalConfig = HALConfig.getGlobalInstance();

        gui = HALGUI.getInstance();

        //Adds opmode program options to the config.
        if (opMode.getClass().isAnnotationPresent(ProgramOptions.class))
            globalConfig.addOpmode(opMode);
    }

    /**
     * Adds a subsystem to the robot's hashmap of subsystems and, if the subsystem uses config, load the default config.
     *
     * @param subSystem The subsystem object.
     *
     * @see SubSystem
     * @see HALConfig
     */
    private void addSubSystem(String name, SubSystem subSystem) {
        subSystems.add(subSystem);

        if(subSystem.usesConfig) {
            boolean success = globalConfig.addSubSystem(name, subSystem);
            useConfig |= success;

            if (success && !gui.isInitialized()) startGui(Button.noButtonBoolean);
        }
    }

    /**
     * Adds a subsystem to the robot's hashmap of subsystems and, if the subsystem uses config, load the default config.
     *
     * @param name The name of the subsystem.
     * @param subSystem The subsystem object.
     * @deprecated Renamed to addSubSystem
     */
    @Deprecated
    protected final void putSubSystem(String name, SubSystem subSystem) {
        addSubSystem(name, subSystem);
    }

    /**
     * Instantiates the GUI and allows the robot to use a GUI.
     *
     * @param cycleButton The button used to cycle through multiple menus in GUI.
     *
     * @see HALGUI
     * @see Button
     */
    public final void startGui(Button<Boolean> cycleButton) {
        if (gui.isInitialized()) gui.setCycleButton(cycleButton);
        else gui.setup(this, cycleButton);
    }

    /**
     * Returns whether the robot has already been set up to use the GUI.
     *
     * @return Whether the GUI has been instantiated.
     *
     * @see HALGUI
     */
    public final boolean usesGUI() {
        return gui.isInitialized();
    }

    /**
     * Runs all the initialization methods of every subsystem and the GUI. Also starts the config and creates the config file tree if needed.
     *
     * @see SubSystem
     * @see HALConfig
     * @see HALGUI
     */
    public final void init()
    {
        //Collects subsystems from robot class and adds them to internal subsystem list.
        Field[] fields = this.getClass().getDeclaredFields();
        for(Field f : fields) {
            if(SubSystem.class.isAssignableFrom(f.getType()) && !f.isAnnotationPresent(DisableSubSystem.class)) {
                Object obj;
                try {
                    obj = f.get(this);
                } catch (IllegalAccessException e) {
                    e.printStackTrace();
                    throw new DumpsterFireException("Tried to access your subsystem, but you made it protected or private. SHARE!!!");
                }
                if(obj != null) {
                    SubSystem subSystem = (SubSystem) obj;
                    String name = subSystem.getClass().getSimpleName();
                    if(f.isAnnotationPresent(ConfigLabel.class)) {
                        ConfigLabel configLabelAnnotation = f.getAnnotation(ConfigLabel.class);
                        name = configLabelAnnotation.label();
                    }
                    addSubSystem(name, subSystem);
                }
            }
        }

        HALConfig.setGlobalConfigAsDefault();

        this.gamepad1 = opMode.gamepad1;
        this.gamepad2 = opMode.gamepad2;

        HALFileUtil.createDirectory(HAL_FILESYSTEM_ROOT);

        if(useConfig) {
            String configMetadataFullName = '/' + ConfigConstants.CONFIG_METADATA_FILENAME + ConfigConstants.CONFIG_FILE_EXTENSION;

            //create overall robot folder
            String robotRootConfigPath = HAL_FILESYSTEM_ROOT + "/robot_" + this.getClass().getSimpleName();
            HALFileUtil.createDirectory(robotRootConfigPath);
            HALFileUtil.createFile(robotRootConfigPath + configMetadataFullName);

            //create autonomous directory in robot folder
            String autonomousFolder = robotRootConfigPath + AUTONOMOUS.filepathExtension;
            HALFileUtil.createDirectory(autonomousFolder);
            HALFileUtil.createFile(autonomousFolder + configMetadataFullName);

            //create teleop directory in robot folder
            String teleopFolder = robotRootConfigPath + TELEOP.filepathExtension;
            HALFileUtil.createDirectory(teleopFolder);
            HALFileUtil.createFile(teleopFolder + configMetadataFullName);

            //Get all names of subsystem objects being used in this robot and write it to the outer robot_info.txt file
            //These names will be used in the config debugger
            StringBuilder sb = new StringBuilder();
            for (SubSystem subSystem : subSystems) {
                if (subSystem.usesConfig) {
                    sb.append(subSystem.getClass().getName());
                    sb.append("\r\n");
                }
            }

            //removes trailing \r\n characters so there isn't a blank line at the end of the file
            if (sb.length() > 2) sb.delete(sb.length() - 2, sb.length());

            HALFileUtil.save(robotRootConfigPath + configMetadataFullName, sb.toString());

            Payload payload = new Payload()
                    .add(ConfigConstants.ROBOT_FILEPATH_ID, robotRootConfigPath)
                    .add(ConfigConstants.SELECTION_MODE_ID, isAutonomous() ? AUTONOMOUS : TELEOP);

            if (opMode.getClass().isAnnotationPresent(StandAlone.class)) {
                payload.add(ConfigConstants.STANDALONE_MODE_ID, true);
            }
            gui.addRootMenu(new ConfigStartingMenu(payload));
        }

        for (int i = 0; i < subSystems.size(); i++) {
            SubSystem subSystem = subSystems.poll();
            subSystem.init();
            subSystems.add(subSystem);
        }
    }

    /**
     * Runs methods in a loop during init. Runs all subsystem init_loop() methods and draws the configuration menu.
     *
     * @see SubSystem
     * @see HALGUI
     */
    public final void init_loop() {
        this.gamepad1 = opMode.gamepad1;
        this.gamepad2 = opMode.gamepad2;

        gui.renderCurrentMenu();

        for (int i = 0; i < subSystems.size(); i++) {
            SubSystem subSystem = subSystems.poll();
            subSystem.init_loop();
            subSystems.add(subSystem);
        }
    }

    /**
     * Runs this method when the user presses the start button.
     *
     * @see SubSystem
     * @see HALGUI
     */
    public final void onStart() {
        this.gamepad1 = opMode.gamepad1;
        this.gamepad2 = opMode.gamepad2;

        for (int i = 0; i < subSystems.size(); i++) {
            SubSystem subSystem = subSystems.poll();
            subSystem.start();
            subSystems.add(subSystem);
        }
    }

    /**
     * Runs subsystem handle() methods and GUI drawCurrentMenu() every frame in driver controlled programs.
     *
     * @see SubSystem
     * @see HALGUI
     */
    public final void driverControlledUpdate() {
        this.gamepad1 = opMode.gamepad1;
        this.gamepad2 = opMode.gamepad2;

        gui.renderCurrentMenu();

        for (int i = 0; i < subSystems.size(); i++) {
            SubSystem subSystem = subSystems.poll();
            subSystem.handle();
            subSystems.add(subSystem);
        }
    }

    /**
     * Runs the stop functions for all subsystems and the GUI.
     *
     * @see SubSystem
     * @see HALGUI
     * @see HALConfig
\     */
    public final void stopAllComponents() {

        gui.stop();

        for (int i = 0; i < subSystems.size(); i++) {
            SubSystem subSystem = subSystems.poll();
            subSystem.stop();
            subSystems.add(subSystem);
        }

        globalConfig.clearConfig();
    }

    /**
     * Gets the opmode the robot is currently running.
     *
     * @return The opmode the robot is running.
     *
     * @see HALProgram
     */
    public final OpMode getOpMode() {
        return opMode;
    }

    /**
     * Pulls a customizable gamepad object from the teleop config map. Allows for easily getting gamepad data from the configuration.
     *
     * @param subsystem The name of the subsystem to pull the gamepad controls for.
     * @return A customizable gamepad containing the configured controls for that subsystem.
     *
     * @see HALConfig
     * @see ConfigParam
     * @see CustomizableGamepad
     * @see Button
     * @see SubSystem
     * @see AutonomousConfig
     * @see TeleopConfig
     */
    public final CustomizableGamepad pullControls(SubSystem subsystem) {
        List<ConfigParam> configParams = globalConfig.getConfig(isAutonomous() ? AUTONOMOUS : TELEOP, subsystem);
        ExceptionChecker.assertNonNull(configParams, new NullPointerException(subsystem + " does not have configurable controls."));

        CustomizableGamepad gamepad = new CustomizableGamepad(this);

        for (ConfigParam param : configParams) {
            if (param.usesGamepad) {
                gamepad.addButton(param.name, param.toButton());
            }
        }

        return gamepad;
    }

    /**
     * Pulls the data of non-gamepad-related config settings from the global config. The map format is (option name) -> (option value)
     *
     * @param subsystem The name of the subsystem to get config from.
     * @return The non-gamepad configuration data for that subsystem.
     *
     * @see HALConfig
     * @see ConfigParam
     * @see ConfigData
     * @see SubSystem
     * @see AutonomousConfig
     * @see TeleopConfig
     */
    public final ConfigData pullNonGamepad(SubSystem subsystem) {
        List<ConfigParam> configParams = globalConfig.getConfig(isAutonomous() ? AUTONOMOUS : TELEOP, subsystem);
        if (configParams == null) configParams = new ArrayList<>();

        Map<String, Object> output = new HashMap<>();

        for (ConfigParam param : configParams) {
            if (!param.usesGamepad) {
                output.put(param.name, param.vals.get(param.options.indexOf(param.currentOption)));
            }
        }

        return new ConfigData(output);
    }

    /**
     * Gets the config settings for the currently running opmode.
     *
     * @return The config settings for the currently running opmode.
     * @see HALConfig
     * @see ConfigParam
     * @see ConfigData
     * @see HALProgram
     * @see ProgramOptions
     */
    public ConfigData pullOpModeSettings() {
        List<ConfigParam> data = globalConfig.getConfig(opMode);
        ExceptionChecker.assertNonNull(data, new NothingToSeeHereException(HALConfig.getOpModeName(opMode.getClass()) + " settings are not part of the config."));

        Map<String, Object> dataMap = new HashMap<>();
        for (ConfigParam param : data) {
            dataMap.put(param.name, param.vals.get(param.options.indexOf(param.currentOption)));
        }

        return new ConfigData(dataMap);
    }

    /**
     * Gets if the program the robot is running is a teleop program.
     *
     * @return Whether the program being run is a teleop program.
     *
     * @see HALProgram
     */
    public final boolean isTeleop() {
        return opMode.getClass().isAnnotationPresent(TeleOp.class);
    }

    /**
     * Gets if the program the robot is running is an autonomous program.
     *
     * @return Whether the program being run is an autonomous program.
     *
     * @see HALProgram
     */
    public final boolean isAutonomous() {
        return opMode.getClass().isAnnotationPresent(Autonomous.class);
    }

    /**
     * Gets whether the robot's current program is running.
     *
     * @return Whether the robot's current program is running.
     *
     * @see HALProgram
     */
    public final boolean opModeIsActive() {
        ExceptionChecker.assertTrue(isTeleop() || isAutonomous(), new DumpsterFireException("Program is not an instance of BaseAutonomous or BaseTeleop, cannot tell if its running. A lot of other things are probably broken too if you're seeing this."));
        return ((LinearOpMode) opMode).opModeIsActive();
    }

    /**
     * Gets whether the robot's current program has requested to be stopped.
     *
     * @return Whether the robot's current program has requested to be stopped.
     *
     * @see HALProgram
     */
    public final boolean isStopRequested() {
        ExceptionChecker.assertTrue(isTeleop() || isAutonomous(), new DumpsterFireException("Program is not an instance of BaseAutonomous or BaseTeleop, cannot tell if its running. A lot of other things are probably broken too if you're seeing this."));
        return ((LinearOpMode) opMode).isStopRequested();
    }

    /**
     * Gets whether the robot's current program has been started.
     *
     * @return Whether the robot's current program has been started.
     *
     * @see HALProgram
     */
    public final boolean isStarted() {
        ExceptionChecker.assertTrue(isTeleop() || isAutonomous(), new DumpsterFireException("Program is not an instance of BaseAutonomous or BaseTeleop, cannot tell if its running. A lot of other things are probably broken too if you're seeing this."));
        return ((LinearOpMode) opMode).isStarted();
    }
}