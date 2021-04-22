package virtual_robot.controller.robots.classes;

import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.hardware.configuration.MotorType;
import javafx.fxml.FXML;
import javafx.scene.shape.Rectangle;
import javafx.scene.transform.Scale;
import javafx.scene.transform.Translate;
import virtual_robot.controller.BotConfig;

/**
 * For internal use only. Represents a robot with four mechanum wheels, color sensor, four distance sensors,
 * a BNO055IMU, and an extensible arm with a grabber on the end.
 *
 * The easiest way to create a new robot configuration is to copy and paste the Java class and the FXML file
 * of an existing configuration, then make make modifications. The ArmBot config is a modification of
 * the MechanumBot config.
 *
 * The @BotConfig annotation is required. The name will be displayed to the user in the Configuration
 * combo box. The filename refers to the fxml file that contains the markup for the graphical UI.
 * Note: the fxml file must be located in the virtual_robot.controller.robots.classes.fxml folder.
 */
//@BotConfig(name = "Arm Bot", filename = "arm_bot")
public class ArmBot extends MechanumBase {

    /*
    The DC Motors.  Note use of the DcMotorImpl class rather than the DcMotor interface. That allows use of
    DcMotorImpl methods (such as update()) that are intended for internal use, and are not part of the
    DcMotor interface. The drive motors are stored in an array of DcMotorImpl.
     */
    private DcMotorExImpl armMotor = null;

    //Servo to control the hand at the end of the arm. Note use of ServoImpl class rather than Servo interface.
    private ServoImpl handServo = null;

    /*
    Variables representing graphical UI nodes that we will need to manipulate. The @FXML annotation will
    cause these variables to be instantiated automatically during loading of the arm_bot.fxml file. The
    fxml file must declare fx:id attributes for the Rectangles that represent the arm, hand, and both fingers.
    For example, the attribute for the arm would be:  fx:id="arm"
     */
    @FXML private Rectangle arm;            //The arm. Must be able to extend/retract (i.e., scale) in Y-dimension.
    @FXML private Rectangle hand;           //The hand. Must move in Y-dimension as arm extends/retracts.
    @FXML private Rectangle leftFinger;     //Fingers must open and close based on position of hand servo.
    @FXML private Rectangle rightFinger;

    /*
    Transform objects that will be instantiated in the initialize() method, and will be used in the
    updateDisplay() method to manipulate the arm, hand, and fingers.
     */
    Scale armScaleTransform;
    Translate handTranslateTransform;
    Translate leftFingerTranslateTransform;
    Translate rightFingerTranslateTransform;

    /*
    Current scale of the arm (i.e., the degree to which arm is extended or retracted). 1.0 means fully retracted.
    2.0 would mean that arm is twice the fully retracted length, etc.
     */
    private double armScale = 1.0;

    /**
     * Constructor.
     */
    public ArmBot() {
        super();
    }

    /**
     *  The initialize() method is called automatically when the robot's graphical UI is loaded from the
     *  arm_bot.fxml markup file. It should be used to set up parts of the graphical UI that will change
     *  as the robot operates
     */
    public void initialize(){
        //This call to the superclass initialize method is essential. Among other things, this will call the
        //createHardwareMap() method, so that a HardwareMap object will be available. It also does all of
        //the initialization of the Mechanum drive base.
        super.initialize();

        //Temporarily activate the hardware map to allow calls to "get"
        hardwareMap.setActive(true);

        armMotor = (DcMotorExImpl)hardwareMap.get(DcMotorEx.class, "arm_motor");

        //Instantiate the hand servo. Note the cast to ServoImpl.
        handServo = (ServoImpl)hardwareMap.servo.get("hand_servo");

        //Deactivate the hardwaremap to prevent users from accessing hardware until after INIT is pressed
        hardwareMap.setActive(false);

        /*
        Scales the arm with pivot point at center of back of robot (which corresponds to the back of the arm).
        The Y-scaling is initialized to 1.0 (i.e., arm fully retracted)
        We will never change the X-scaling (we don't need the arm to get fatter, only longer)
         */
        armScaleTransform = new Scale(1.0, 1.0, 37.5, 75.0);
        arm.getTransforms().add(armScaleTransform);

        // Translates the position of the hand so that it stays at the end of the arm.
        handTranslateTransform = new Translate(0, 0);
        hand.getTransforms().add(handTranslateTransform);

        /*
        Translates the position of the fingers in both X and Y dimensions. The X-translation is to open and close
        the fingers. The Y-translation is so the fingers move along with the arm and hand, as the arm extends.
         */
        leftFingerTranslateTransform = new Translate(0, 0);
        leftFinger.getTransforms().add(leftFingerTranslateTransform);

        rightFingerTranslateTransform = new Translate(0, 0);
        rightFinger.getTransforms().add(rightFingerTranslateTransform);
    }

    /**
     * Create the HardwareMap object
     */
    protected void createHardwareMap() {
        super.createHardwareMap();

        //Add the arm motor using HardwareMap.put(...) method
        hardwareMap.put("arm_motor", new DcMotorExImpl(MotorType.Neverest40));

        //Add the ServoImpl object
        hardwareMap.put("hand_servo", new ServoImpl());
    }

    /**
     * Update robot position on field and update the robot sensors
     * @param millis milliseconds since the previous update
     */
    public synchronized void updateStateAndSensors(double millis){
        super.updateStateAndSensors(millis);

        /*
        Calculate the new value of armScale based on interval motion of the arm motor. BUT, do not manipulate
        the arm UI here. Do that in the updateDisplay method, which will ultimately be called from the UI Thread.
         */
        double armTicks = armMotor.update(millis);
        double newArmScale = armScale + armTicks / 1120.;
        if (newArmScale >= 1.0 && newArmScale <= 4) armScale = newArmScale;
    }

    /**
     *  Update the display of the robot UI. This method will be called from the UI Thread via a call to
     *  Platform.runLater().
     */
    @Override
    public synchronized void updateDisplay(){
        /*
        This call to super.updateDisplay() is essential. the superclass method puts the robot in the correct
        position and orientation on the field.
         */
        super.updateDisplay();

        // Extend or retract the arm based on the value of armScale.

        if (Math.abs(armScale - armScaleTransform.getY()) > 0.001) {
            armScaleTransform.setY(armScale);

            // Move the hand based on the position of armScale.
            handTranslateTransform.setY(-40.0 * (armScale - 1.0));

            // Move the fingers in the Y-direction based on the position of armScale.
            leftFingerTranslateTransform.setY(-40.0 * (armScale - 1.0));
            rightFingerTranslateTransform.setY(-40.0 * (armScale - 1.0));
        }

        // Mover fingers in the X-direction (i.e., open/close fingers) based on position of the handServo.
        double fingerPos =  7.5 * handServo.getInternalPosition();
        if (Math.abs(fingerPos - leftFingerTranslateTransform.getX()) > 0.001) {
            leftFingerTranslateTransform.setX(fingerPos);
            rightFingerTranslateTransform.setX(-fingerPos);
        }

    }

    /**
     *  Stop all motors and close the BNO055IMU
     */
    public void powerDownAndReset(){
        super.powerDownAndReset();
        armMotor.stopAndReset();
    }
}
