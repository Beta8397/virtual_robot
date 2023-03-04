package virtual_robot.robots.classes;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorExImpl;
import com.qualcomm.robotcore.hardware.ServoImpl;
import com.qualcomm.robotcore.hardware.configuration.MotorType;
import javafx.fxml.FXML;
import javafx.scene.Group;
import javafx.scene.input.MouseEvent;
import javafx.scene.shape.Rectangle;
import javafx.scene.shape.Shape;
import javafx.scene.transform.Scale;
import javafx.scene.transform.Translate;
import org.dyn4j.collision.CategoryFilter;
import org.dyn4j.dynamics.Body;
import org.dyn4j.geometry.Transform;
import org.dyn4j.geometry.Vector2;
import virtual_robot.controller.BotConfig;
import virtual_robot.controller.Filters;
import virtual_robot.controller.VirtualField;
import virtual_robot.dyn4j.Dyn4jUtil;
import virtual_robot.dyn4j.FixtureData;
import virtual_robot.dyn4j.Slide;
import virtual_robot.game_elements.classes.Freight;
import virtual_robot.game_elements.classes.WobbleGoal;

import java.util.HashMap;

/**
 * For internal use only. Represents a robot with four mecanum wheels, color sensor, four distance sensors,
 * a BNO055IMU, and an extensible arm with a grabber on the end.
 *
 * ArmBot extends the MecanumPhysicsBase class, which manages the drive train and various sensors, as well as
 * basic interaction between the chassis and the walls and game elements.
 *
 * ArmBot adds the extensible arm, including both the graphical display and the dyn4j Bodys that represent the
 * arm. Any "special" interaction between the robot and game elements must be handled by ArmBot.
 *
 * The @BotConfig annotation is required. The name will be displayed to the user in the Configuration
 * combo box. The filename refers to the fxml file that contains the markup for the graphical UI.
 * Note: the fxml file must be located in the virtual_robot.robots.classes.fxml folder.
 */
@BotConfig(name = "Arm Bot", filename = "arm_bot")
public class ArmBot extends MecanumPhysicsBase {

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
    @FXML private Group armGroup;
    @FXML private Rectangle arm;
    @FXML private Rectangle hand;
    @FXML private Group leftFingerGroup;
    @FXML private Rectangle leftProximalPhalanx;
    @FXML private Rectangle leftDistalPhalanx;
    @FXML private Group rightFingerGroup;
    @FXML private Rectangle rightProximalPhalanx;
    @FXML private Rectangle rightDistalPhalanx;

    /*
     * Transform objects that will be instantiated in the initialize() method, and will be used in the
     * updateDisplay() method to manipulate the arm, hand, and fingers.
     */
    Translate armTranslateTransform;
    Translate leftFingerTranslateTransform;
    Translate rightFingerTranslateTransform;

    /*
     * Current Y-translation of the arm, in pixels. 0 means fully retracted. 50 means fully extended.
     */
    private double armTranslation = 0;

    /*
     * Current X-translation of the fingers, in pixels. 0 means fully open.
     */
    private double fingerPos = 0;

    private Body armBody;
    private Body leftFingerBody;
    private Body rightFingerBody;
    Slide armSlide;
    Slide leftFingerSlide;
    Slide rightFingerSlide;

    private CategoryFilter ARM_FILTER = new CategoryFilter(Filters.ARM, WobbleGoal.WOBBLE_HANDLE_CATEGORY | Filters.WALL);

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
        armMotor.setActualPositionLimits(0, 2240);
        armMotor.setPositionLimitsEnabled(true);

        //Instantiate the hand servo. Note the cast to ServoImpl.
        handServo = (ServoImpl)hardwareMap.servo.get("hand_servo");

        //Deactivate the hardwaremap to prevent users from accessing hardware until after INIT is pressed
        hardwareMap.setActive(false);

        /*
         * Translates the arm (only the Y-dimension will be used).
         */
        armTranslateTransform = new Translate(0, 0);
        armGroup.getTransforms().add(armTranslateTransform);

        /*
         * Translates the position of the fingers in both X and Y dimensions. The X-translation is to open and close
         * the fingers. The Y-translation is so the fingers move along with the arm and hand, as the arm extends.
         */
        leftFingerTranslateTransform = new Translate(0, 0);
        leftFingerGroup.getTransforms().add(leftFingerTranslateTransform);

        rightFingerTranslateTransform = new Translate(0, 0);
        rightFingerGroup.getTransforms().add(rightFingerTranslateTransform);

        /*
         * Create the dyn4j Body for the arm; it will have two BodyFixtures, one corresponding to the "arm" javafx ,
         * Shape, and the other to the "hand" shape. Each of those will be created using a different FixtureData, to allow the
         * dyn4j shapes to have twice the short-axis thickness of the very thin javafx shapes. Then create a Slide
         * joint that connects the arm body to the chassis body. The slide joint will have a default unit of
         * PIXEL, so that its position can be set by passing in pixels directly.
         */
        HashMap<Shape, FixtureData> armMap = new HashMap<>();
        armMap.put(arm, new FixtureData(ARM_FILTER,1.0, 0, 0.25, 2, 1));
        armMap.put(hand, new FixtureData(ARM_FILTER, 1.0, 0, 0.25, 1, 2));
        armBody = Dyn4jUtil.createBody(armGroup, this, 9, 9, armMap);
        world.addBody(armBody);
        armSlide = new Slide(chassisBody, armBody, new Vector2(0,0), new Vector2(0,-1),
                VirtualField.Unit.PIXEL);
        world.addJoint(armSlide);

        /*
         * Create dyn4j Bodys for each finger. Each body will have two BodyFixtures (proximal and distal phalanges).
         * Each of those BodyFixtures will be created using a different FixtureData object, to allow the dyn4j shapes
         * to have twice the short-axis thickness of the very thin javafx shapes. Then for each finger, create a
         * Slide joint that connects the finger body to the arm body.
         */
        HashMap<Shape, FixtureData> leftFingerMap = new HashMap();
        leftFingerMap.put(leftProximalPhalanx, new FixtureData(ARM_FILTER,1.0, 0, 0.25, 2, 1));
        leftFingerMap.put(leftDistalPhalanx, new FixtureData(ARM_FILTER, 1.0, 0, 0.25, 1, 2));
        leftFingerBody = Dyn4jUtil.createBody(leftFingerGroup, this, 9, 9, leftFingerMap);
        world.addBody(leftFingerBody);
        leftFingerSlide = new Slide(armBody, leftFingerBody, new Vector2(0,0), new Vector2(-1, 0),
                VirtualField.Unit.PIXEL);
        world.addJoint(leftFingerSlide);

        HashMap<Shape, FixtureData> rightFingerMap = new HashMap();
        rightFingerMap.put(rightProximalPhalanx, new FixtureData(ARM_FILTER,1.0, 0, 0.25, 2, 1));
        rightFingerMap.put(rightDistalPhalanx, new FixtureData(ARM_FILTER, 1.0, 0, 0.25, 1, 2));
        rightFingerBody = Dyn4jUtil.createBody(rightFingerGroup, this, 9, 9, rightFingerMap);
        world.addBody(rightFingerBody);
        rightFingerSlide = new Slide(armBody, rightFingerBody, new Vector2(0, 0), new Vector2(-1, 0),
                VirtualField.Unit.PIXEL);
        world.addJoint(rightFingerSlide);
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
        super.updateStateAndSensors(millis);    // Handles update for drivetrain and standard sensors.

        /*
         * Update the arm motor and calculate the new value of armTranslation (in pixels). Then, set the position
         * of the arm slide object to armTranslation, in pixels. This positions the arm in the dyn4j physics engine.
         * It does not handle the display, of the arm--that will be done in the updateDisplay method, using the
         * new value of armTranslation.
         */
        armMotor.update(millis);
        armTranslation = armMotor.getActualPosition() * 50.0 / 2240.0 * (botWidth / 75.0);
        armSlide.setPosition(armTranslation);

        /*
         * Update the value of fingerPos, using the current position of the hand servo. Then set the position
         * of each finger slide joint using this value.
         */
        fingerPos =  15 * handServo.getInternalPosition();
        leftFingerSlide.setPosition(fingerPos);
        rightFingerSlide.setPosition(-fingerPos);

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

        armTranslateTransform.setY(-armTranslation);
        leftFingerTranslateTransform.setY(-armTranslation);
        rightFingerTranslateTransform.setY(-armTranslation);

        // Mover fingers in the X-direction (i.e., open/close fingers) based on position of the handServo.

        if (Math.abs(fingerPos - leftFingerTranslateTransform.getX()) > 0.001) {
            leftFingerTranslateTransform.setX(fingerPos);
            rightFingerTranslateTransform.setX(-fingerPos);
        }


    }

    /**
     * The superclass positionWithMouseClick method only positions the chassis. We need to position the
     * arm and fingers as well, and set their velocities (linear, angular), accumulated forces, and
     * accumulated torques to zero.
     *
     * @param arg
     */
    @Override
    public synchronized void positionWithMouseClick(MouseEvent arg){

        /*
         * Get the transforms of the arm and fingers (and any loaded Freight item) relative to the chassis
         */
        Transform tArmChassis = Dyn4jUtil.multiplyTransforms(Dyn4jUtil.getInverseTransform(chassisBody.getTransform()),
                armBody.getTransform());
        Transform tLeftFingerChassis = Dyn4jUtil.multiplyTransforms(Dyn4jUtil.getInverseTransform(chassisBody.getTransform()),
                leftFingerBody.getTransform());
        Transform tRightFingerChassis = Dyn4jUtil.multiplyTransforms(Dyn4jUtil.getInverseTransform(chassisBody.getTransform()),
                rightFingerBody.getTransform());

        /*
         * the super method repositions the chassis based on mouse click
         */
        super.positionWithMouseClick(arg);

        /*
         * Determine the new transforms of arm and fingers relative to the field
         */
        Transform tArm = Dyn4jUtil.multiplyTransforms(chassisBody.getTransform(), tArmChassis);
        Transform tLeftFinger = Dyn4jUtil.multiplyTransforms(chassisBody.getTransform(), tLeftFingerChassis);
        Transform tRightFinger = Dyn4jUtil.multiplyTransforms(chassisBody.getTransform(), tRightFingerChassis);
        armBody.setTransform(tArm);
        armBody.setLinearVelocity(0,0);
        armBody.setAngularVelocity(0);
        armBody.clearAccumulatedForce();
        armBody.clearAccumulatedTorque();
        leftFingerBody.setTransform(tLeftFinger);
        leftFingerBody.setLinearVelocity(0,0);
        leftFingerBody.setAngularVelocity(0);
        leftFingerBody.clearAccumulatedTorque();
        leftFingerBody.clearAccumulatedForce();;
        rightFingerBody.setTransform(tRightFinger);
        rightFingerBody.setLinearVelocity(0,0);
        rightFingerBody.setAngularVelocity(0);
        rightFingerBody.clearAccumulatedForce();
        rightFingerBody.clearAccumulatedTorque();

    }

    /**
     *  Stop all motors and close the BNO055IMU
     */
    public void powerDownAndReset(){
        super.powerDownAndReset();
        armMotor.stopAndReset();
    }

}
