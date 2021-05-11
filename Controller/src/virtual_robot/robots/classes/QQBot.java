package virtual_robot.robots.classes;

import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.hardware.configuration.MotorType;

/**
 * For internal use only. Represents a robot with four mechanum wheels, color sensor, four distance sensors,
 * a BNO055IMU, and a turret that rotates and elevates.
 * <p>
 * The easiest way to create a new robot configuration is to copy and paste the Java class and the FXML file
 * of an existing configuration, then make make modifications. The ArmBot config is a modification of
 * the MechanumBot config.
 * <p>
 * The @BotConfig annotation is required. The name will be displayed to the user in the Configuration
 * combo box. The filename refers to the fxml file that contains the markup for the graphical UI.
 * Note: the fxml file must be located in the virtual_robot.robots.classes.fxml folder.
 */
//@BotConfig(name = "QQ Bot", filename = "qq_bot")
public class QQBot extends TurretBot {
    // Wobbly goal mechanism
    private ServoImpl grabberServo;
    private ServoImpl rotatorServo;
    // intake mechanism
    private DcMotorExImpl intakeMotor;

    /**
     * Constructor.
     */
    public QQBot() {
        super();

    }

    /**
     * The initialize() method is called automatically when the robot's graphical UI is loaded from the
     * arm_bot.fxml markup file. It should be used to set up parts of the graphical UI that will change
     * as the robot operates
     */
    public void initialize() {
        super.initialize();
        //Temporarily activate the hardware map to allow calls to "get"
        hardwareMap.setActive(true);

        //Instantiate the wobbly goal servos. Note the cast to ServoImpl.
        grabberServo = (ServoImpl) hardwareMap.servo.get("grabber");
        rotatorServo = (ServoImpl) hardwareMap.servo.get("rotator");

        //Instantiate the motor
        intakeMotor = (DcMotorExImpl) hardwareMap.dcMotor.get("intake_motor");

        //Deactivate the hardwaremap to prevent users from accessing hardware until after INIT is pressed
        hardwareMap.setActive(false);

        gearRatioWheel = 0.5;  // take into account 2:1 reduction from motor
    }

    /**
     * Create the HardwareMap object
     */
    protected void createHardwareMap() {
        super.createHardwareMap();


        String[] motorNames = new String[]{"back_left_motor", "front_left_motor", "front_right_motor", "back_right_motor"};

        hardwareMap.setActive(true);
        /*
         * Removing the motors from the dcMotor DeviceMapping removes all trace of them from the HardwareMap (from
         * the DeviceMapping inner class instance and from the HardwareMap outer class instance). Using the remove method
         * of HardwareMap directly does not remove them from the DeviceMapping.
         */
        for (String name : motorNames) {
            hardwareMap.dcMotor.remove(name);
        }
        hardwareMap.setActive(false);
        for (String name : motorNames) hardwareMap.put(name, new DcMotorExImpl(MotorType.Gobilda137));

        /*
         * Note: this will overwrite the ColorSensor object that is already in the HardwareMap from the
         * MechanumBase class. If your op mode obtains a reference to this new color sensor, it won't function
         * (because it isn't being updated in the updateStateAndSensors method of QQBot). If you want a second color
         * sensor, you'd need to use a different name (e.g., "color_sensor_2"), and would then need to handle
         * updating it in the updateStateAndSensors method.
         */
//        hardwareMap.put("color_sensor", controller.new ColorSensorImpl());

        hardwareMap.put("grabber", new ServoImpl());
        hardwareMap.put("rotator", new ServoImpl());

        hardwareMap.put("intake_motor", new DcMotorExImpl(MotorType.Neverest40));

        hardwareMap.put("transfer_motor", new DcMotorExImpl(MotorType.Neverest40));

        hardwareMap.put("shooter_back_motor", new DcMotorExImpl(MotorType.Neverest40));
        hardwareMap.put("shooter_front_motor", new DcMotorExImpl(MotorType.Neverest40));

        hardwareMap.put("servo_pivot_shooter", new ServoImpl());
        hardwareMap.put("servo_import_shooter", new ServoImpl());
    }

    /**
     * Update robot position on field and update the robot sensors
     *
     * @param millis milliseconds since the previous update
     */
    public synchronized void updateStateAndSensors(double millis) {
        super.updateStateAndSensors(millis);
    }

    /**
     * Update the display of the robot UI. This method will be called from the UI Thread via a call to
     * Platform.runLater().
     */
    @Override
    public synchronized void updateDisplay() {
        super.updateDisplay();
        // should probably have something here about a grabber...
    }

    /**
     * Stop all motors and close the BNO055IMU
     */
    public void powerDownAndReset() {
        super.powerDownAndReset();
    }

}
