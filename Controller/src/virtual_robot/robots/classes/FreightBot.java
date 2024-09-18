package virtual_robot.robots.classes;

import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.hardware.configuration.MotorType;
import javafx.fxml.FXML;
import javafx.scene.Group;
import javafx.scene.input.MouseButton;
import javafx.scene.input.MouseEvent;
import javafx.scene.shape.Circle;
import javafx.scene.shape.Rectangle;
import javafx.scene.shape.Shape;
import javafx.scene.transform.Rotate;
import javafx.scene.transform.Translate;
import org.dyn4j.collision.CategoryFilter;
import org.dyn4j.dynamics.Body;
import org.dyn4j.dynamics.BodyFixture;
import org.dyn4j.dynamics.joint.RevoluteJoint;
import org.dyn4j.dynamics.joint.WeldJoint;
import org.dyn4j.geometry.Transform;
import org.dyn4j.geometry.Vector2;
import org.dyn4j.world.NarrowphaseCollisionData;
import org.dyn4j.world.listener.CollisionListenerAdapter;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import virtual_robot.controller.BotConfig;
import virtual_robot.controller.Filters;
import virtual_robot.controller.Game;
import virtual_robot.controller.VirtualField;
import virtual_robot.dyn4j.Dyn4jUtil;
import virtual_robot.dyn4j.FixtureData;
import virtual_robot.dyn4j.Slide;
import virtual_robot.game_elements.classes.*;
import virtual_robot.games.FreightFrenzy;
import virtual_robot.robots.ControlsElements;

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
//@BotConfig(name = "Freight Bot", filename = "freight_bot")
public class FreightBot extends MecanumPhysicsBase implements ControlsElements {

    /*
    The DC Motors.  Note use of the DcMotorImpl class rather than the DcMotor interface. That allows use of
    DcMotorImpl methods (such as update()) that are intended for internal use, and are not part of the
    DcMotor interface. The drive motors are stored in an array of DcMotorImpl.
     */
    private DcMotorExImpl armMotor = null;
    private DcMotorExImpl rotorMotor = null;

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
    @FXML private Group rightFingerGroup;
    @FXML private Rectangle armSensorRect;
    @FXML private Circle rotorCircle;
    @FXML private Group rotorGroup;

    /*
     * Transform objects that will be instantiated in the initialize() method, and will be used in the
     * updateDisplay() method to manipulate the arm, hand, and fingers.
     */
    Translate armTranslateTransform;
    Translate leftFingerTranslateTransform;
    Translate rightFingerTranslateTransform;
    Rotate rotorGroupRotateTransform;

    /*
     * Current Y-translation of the arm, in pixels. 0 means fully retracted. 50 means fully extended.
     */
    private double armTranslation = 0;

    /*
     * Current X-translation of the fingers, in pixels. 0 means fully open.
     */
    private double fingerPos = 0;

    /*
     * Fields to handle dyn4j physics representation of the arm/fingers
     */
    private Body armBody;                   // Will have BodyFixtures for arm and hand
    private Body leftFingerBody;
    private Body rightFingerBody;
    private Slide armSlide;                 // Slide connects arm to chassisBoy
    private Slide leftFingerSlide;          // Slide connects left finger to arm
    private Slide rightFingerSlide;         // Slide connects right finger to arm
    private BodyFixture armSensorFixture;   // Sensor fixture to detect Freight item between fingers

    /*
     * Fields to handle loading of freight when fingers close around it
     */
    private Freight freightToLoad = null;
    private Freight loadedFreight = null;
    private boolean fingersClosed = false;
    private WeldJoint<Body> loadedFreightJoint = null;

    private Body rotorBody;
    private RevoluteJoint rotorJoint;


    private CategoryFilter ARM_FILTER = new CategoryFilter(Filters.ARM,
            Filters.MASK_ALL & ~Barrier.BARRIER_CATEGORY & ~ShippingHub.HUB_CATEGORY
                    & ~Filters.CHASSIS & ~Filters.ARM & ~Backdrop.BACKDROP_CATEGORY);

    private CategoryFilter ROTOR_FILTER = new CategoryFilter(Carousel.CAROUSEL_SPINNER_CATEGORY, Carousel.CAROUSEL_CATEGORY);

    /**
     * Constructor.
     */
    public FreightBot() {
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

        rotorMotor = hardwareMap.get(DcMotorExImpl.class, "rotor_motor");

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

        armSensorFixture = Dyn4jUtil.createFixture(armSensorRect, 9, 9, true,
                new FixtureData(ARM_FILTER, true, 1.0, 1.0));
        armBody.addFixture(armSensorFixture);

        world.addBody(armBody);
        armSlide = new Slide(chassisBody, armBody, new Vector2(0,0), new Vector2(0,-1),
                VirtualField.Unit.PIXEL);
        world.addJoint(armSlide);

        /*
         * Create dyn4j Bodys for each finger. Then for each finger create a slide joint connecting finger to arm
         */
        FixtureData fingerFixtureData = new FixtureData(ARM_FILTER, 1.0, 0, 0.25, 2, 1);

        leftFingerBody = Dyn4jUtil.createBody(leftFingerGroup, this, 9, 9, fingerFixtureData);
        world.addBody(leftFingerBody);
        leftFingerSlide = new Slide(armBody, leftFingerBody, new Vector2(0,0), new Vector2(-1, 0),
                VirtualField.Unit.PIXEL);
        world.addJoint(leftFingerSlide);

        rightFingerBody = Dyn4jUtil.createBody(rightFingerGroup, this, 9, 9, fingerFixtureData);
        world.addBody(rightFingerBody);
        rightFingerSlide = new Slide(armBody, rightFingerBody, new Vector2(0, 0), new Vector2(-1, 0),
                VirtualField.Unit.PIXEL);
        world.addJoint(rightFingerSlide);

        FixtureData rotorFixtureData = new FixtureData(ROTOR_FILTER, 1.0, 0, 1.0, 1, 1);
        rotorBody = Dyn4jUtil.createBody(rotorCircle, this, 9, 9, rotorFixtureData);
        world.addBody(rotorBody);
        rotorJoint = new RevoluteJoint(rotorBody, chassisBody, rotorBody.getTransform().getTranslation());
        rotorJoint.setMotorEnabled(true);
        rotorJoint.setMaximumMotorTorque(100);
        rotorJoint.setMotorSpeed(0);
        world.addJoint(rotorJoint);

        rotorGroupRotateTransform = new Rotate(0, 22, 63);
        rotorGroup.getTransforms().add(rotorGroupRotateTransform);


        /*
         * Add a collision listener to the dyn4j world. This will handle collisions where the bot needs
         * to take control of a game element.
         */
        world.addCollisionListener(new CollisionListenerAdapter<Body, BodyFixture>(){
            @Override
            public boolean collision(NarrowphaseCollisionData<Body, BodyFixture> collision) {
                return handleNarrowPhaseCollisions(collision);
            }
        });
    }

    /**
     * Create the HardwareMap object
     */
    protected void createHardwareMap() {
        super.createHardwareMap();

        //Add the arm motor using HardwareMap.put(...) method
        hardwareMap.put("arm_motor", new DcMotorExImpl(MotorType.Neverest40, motorController1, 0));

        //Add the ServoImpl object
        hardwareMap.put("hand_servo", new ServoImpl());

        //Add the Rotor motor
        hardwareMap.put("rotor_motor", new DcMotorExImpl(MotorType.Neverest40, motorController1, 1));
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
         * Check speed of the rotor motor, and use it to adjust the speed of the rotor joint motor
         */
        rotorMotor.update(millis);
        double rotorMotorSpeed = rotorMotor.getVelocity(AngleUnit.RADIANS);
        rotorJoint.setMotorSpeed(rotorMotorSpeed);

        /*
         * Save prior value of fingersClosed, then
         * Update the value of fingerPos, using the current position of the hand servo. Then set the position
         * of each finger slide joint using this value, then
         * Set new value of fingersClosed
         */
        boolean priorFingersClosed = fingersClosed;
        fingerPos =  15 * handServo.getInternalPosition();
        leftFingerSlide.setPosition(fingerPos);
        rightFingerSlide.setPosition(-fingerPos);
        fingersClosed = fingerPos > 10;

        /*
         * If fingers have transitioned from open to closed, check for freightToLoad, and if not null, load the
         * freight by creating a fixed joint between the arm and the freight.
         * But, if fingers have transitioned from closed to open, check for a loaded freight. If one exists then set
         * loadedFreight to null and delete the fixed joint between the arm and the loaded freight.
         * Finally, leave freightToLoad null.
         */


        if (!priorFingersClosed && fingersClosed) {
            System.out.println("Fingers just closed");if (loadedFreight != null) System.out.println("Loaded Freight");
            if (freightToLoad != null) System.out.println("FreightToLoad");
        }

        if (!priorFingersClosed && fingersClosed && freightToLoad != null && loadedFreight == null){
            loadFreight(freightToLoad);
        } else if (priorFingersClosed && !fingersClosed && loadedFreight != null){
            Freight releasedFreight = releaseFreight();
            Vector2 releasedFreightTranslation = releasedFreight.getElementBody().getTransform().getTranslation();
            for (ShippingHub hub: ShippingHub.shippingHubs){
                Vector2 hubTranslation = hub.getElementBody().getTransform().getTranslation();
                double hubDist = releasedFreightTranslation.distance(hubTranslation) * VirtualField.INCHES_PER_METER;
                if (hubDist < ShippingHub.SHIPPING_HUB_RADIUS){
                    releasedFreight.setOwningShippingHub(hub);
                    break;
                }
            }
        }

        freightToLoad = null;

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

        rotorGroupRotateTransform.setAngle(-rotorMotor.getCurrentPosition()*360.0/1120.0);


    }

    /**
     *  Stop all motors and close the BNO055IMU
     */
    public void powerDownAndReset(){
        super.powerDownAndReset();
        armMotor.stopAndReset();
    }


    /**
     * Listener method to handle Narrowphase collision. This method will look specifically for collisions that cause
     * the robot to control a previously un-controlled game element. This method is called DURING the world
     * update.
     *
     * @param collision
     * @return True to allow collision resolution to continue; False to terminate collision resolution.
     */
    private boolean handleNarrowPhaseCollisions(NarrowphaseCollisionData<Body, BodyFixture> collision){
        BodyFixture f1 = collision.getFixture1();
        BodyFixture f2 = collision.getFixture2();

        if ((f1 == armSensorFixture) || (f2 == armSensorFixture) && freightToLoad == null && loadedFreight == null
                && fingersClosed == false) {
            Body b = f1 == armSensorFixture? collision.getBody2() : collision.getBody1();
            if (b.getUserData() instanceof Freight){
                freightToLoad = (Freight)b.getUserData();
            }
            return false;
        }

        return true;
    }

    /**
     * If there is currently no loaded freight item, then load the specified freight item.
     * Also, if the fingers are not currently closed, then close them.
     * This does not automatically update the display of the robot or the freight item; it the
     * physics engine isn't running, that must be done separately, after the call to loadFreight.
     * @param freight
     */
    private void loadFreight(Freight freight){
        if (loadedFreight != null) return;
        freight.setOwningShippingHub(null);
        Transform armTransform = armBody.getTransform();
        Vector2 armTranslation = armTransform.getTranslation();
        Vector2 handOffset = new Vector2(-armTransform.getSint(), armTransform.getCost())
                .product(8.0 / VirtualField.INCHES_PER_METER);
        Vector2 newFreightTranslation = armTranslation.sum(handOffset);
        freight.setOnField(false);
        freight.setLocationMeters(newFreightTranslation);
        freight.setOnField(true);
        loadedFreight = freight;
        loadedFreightJoint = new WeldJoint(armBody, loadedFreight.getElementBody(), armTranslation);
        world.addJoint(loadedFreightJoint);
        loadedFreight.setCategoryFilter(Freight.OWNED_FILTER);
        if (!fingersClosed){
            handServo.setInternalPosition(0.7);
            fingerPos = 15 * 0.7;
            fingersClosed = true;
        }
    }

    private Freight releaseFreight(){
        if (loadedFreight == null) return null;
        world.removeJoint(loadedFreightJoint);
        loadedFreightJoint = null;
        loadedFreight.setCategoryFilter(Freight.NORMAL_FILTER);
        Freight tempFreight = loadedFreight;
        loadedFreight = null;
        return tempFreight;
    }

    @Override
    public void preloadElements(Game game) {
        if ( !(game instanceof FreightFrenzy) ) return;
        if ( loadedFreight == null ) {
            Freight freight = BoxFreight.boxes.get(BoxFreight.boxes.size()-1);
            loadFreight(freight);
            freight.updateDisplay();
            this.updateDisplay();
        }
    }

    @Override
    public void clearLoadedElements(Game game) {
        if ( !(game instanceof FreightFrenzy)) return;
        if (loadedFreight != null) releaseFreight();
        freightToLoad = null;
    }

    /**
     * Position the FreightBot on the field using a MouseEvent.
     *
     * This override of the VirtualBot method is necessary so that the armBody will move appropriately with
     * mouse-click repositioning, even though the physics engine isn't running at that time.
     *
     * @param arg
     */
    @Override
    public synchronized void positionWithMouseClick(MouseEvent arg){

        Freight freight = null;

        /*
         * Release any loaded freight before repositioning
         */
        if (loadedFreight != null) freight = releaseFreight();

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

        if (freight != null) {
            loadFreight(freight);
            freight.updateDisplay();
        }
    }
}
