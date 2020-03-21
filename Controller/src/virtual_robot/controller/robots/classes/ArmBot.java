package virtual_robot.controller.robots.classes;

import com.qualcomm.hardware.bosch.BNO055IMUImpl;
import com.qualcomm.robotcore.hardware.DcMotorImpl;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.MotorType;
import com.qualcomm.robotcore.hardware.ServoImpl;
import javafx.fxml.FXML;
import javafx.scene.shape.Rectangle;
import javafx.scene.transform.Scale;
import javafx.scene.transform.Translate;
import virtual_robot.controller.BotConfig;
import virtual_robot.controller.VirtualBot;
import virtual_robot.controller.VirtualRobotController;
import virtual_robot.util.AngleUtils;

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
@BotConfig(name = "Arm Bot", filename = "arm_bot")
public class ArmBot extends VirtualBot {

    /*
    Motor type to use for the DC Motors in this bot. This has to be instantiated in the createHardwareMap()
    method, because createHardwareMap() is called by the VirtualBot constructor, before the initialization code
    here would be run.
     */
    public MotorType motorType;

    /*
    The DC Motors.  Note use of the DcMotorImpl class rather than the DcMotor interface. That allows use of
    DcMotorImpl methods (such as update()) that are intended for internal use, and are not part of the
    DcMotor interface. The drive motors are stored in an array of DcMotorImpl.
     */
    private DcMotorImpl[] motors = null;
    private DcMotorImpl armMotor = null;

    //The BNO055 IMU. Note use of the BNO055IMUImpl class, rather than the BNO055IMU interface.
    private BNO055IMUImpl imu = null;

    //Sensors.  Note again use of the ...Impl classes, rather than the interfaces.
    private VirtualRobotController.ColorSensorImpl colorSensor = null;
    private VirtualRobotController.DistanceSensorImpl[] distanceSensors = null;

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

    /*
    Final variables that describe robot geometry. They are needed for Odometry (i.e., tracking the position and
    orientation of the robot on the field. Their values will depend on the value of VirtualBot.botWidth. That
    variable is assigned in the VirtualBot constructor. So, the values of these variables will be assigned in
    the ArmBot() constructor (which has a call to the VirtualBot constructor as its first statement).
     */
    private final double WHEEL_CIRCUMFERENCE;
    private final double INTER_WHEEL_WIDTH;
    private double INTER_WHEEL_LENGTH;
    private final double WIDTH_LENGTH_AVERAGE;

    /*
    This matrix will be instantiated in the ArmBot constructor. It is the matrix that transforms from
    Small changes in encoder ticks of the drive wheels to actual changes in robot position and
    orientation.
     */
    private double[][] tWR; //Transform from wheel motion to robot motion

    /**
     * Constructor.
     */
    public ArmBot() {

        //This call to the superclass constructor is essential. Among other things, this will call the
        //createHardwareMap() method, so that a HardwareMap object will be available.
        super();

        //Instantiate the DC Motors using the HardwareMap object. Note the cast to DcMotorImpl.
        motors = new DcMotorImpl[]{
                (DcMotorImpl)hardwareMap.dcMotor.get("back_left_motor"),
                (DcMotorImpl)hardwareMap.dcMotor.get("front_left_motor"),
                (DcMotorImpl)hardwareMap.dcMotor.get("front_right_motor"),
                (DcMotorImpl)hardwareMap.dcMotor.get("back_right_motor")
        };

        armMotor = (DcMotorImpl)hardwareMap.dcMotor.get("arm_motor");

        //Instantiate the four DistanceSensors
        distanceSensors = new VirtualRobotController.DistanceSensorImpl[]{
                hardwareMap.get(VirtualRobotController.DistanceSensorImpl.class, "front_distance"),
                hardwareMap.get(VirtualRobotController.DistanceSensorImpl.class, "left_distance"),
                hardwareMap.get(VirtualRobotController.DistanceSensorImpl.class, "back_distance"),
                hardwareMap.get(VirtualRobotController.DistanceSensorImpl.class, "right_distance")
        };

        //Instantiate the hand servo. Note the cast to ServoImpl.
        handServo = (ServoImpl)hardwareMap.servo.get("hand_servo");

        //Instantiate the BNO055IMUImpl object using the HardwareMap.
        imu = hardwareMap.get(BNO055IMUImpl.class, "imu");

        //Instantiate the ColorSensorImpl object using the HardwareMap.
        colorSensor = (VirtualRobotController.ColorSensorImpl)hardwareMap.colorSensor.get("color_sensor");

        //Assign values to the final variables describing robot geometry
        WHEEL_CIRCUMFERENCE = Math.PI * botWidth / 4.5;
        INTER_WHEEL_WIDTH = botWidth * 8.0 / 9.0;
        INTER_WHEEL_LENGTH = botWidth * 7.0 / 9.0;
        WIDTH_LENGTH_AVERAGE = (INTER_WHEEL_LENGTH + INTER_WHEEL_WIDTH) / 2.0;

        /* Instantiate the Transform Matrix. This matrix converts from incremental distances moved by each robot wheel
        and the corresponding small changes in robot position and orientation in the robot's coordinate system.
        */
        tWR = new double[][] {
                {-0.25, 0.25, -0.25, 0.25},
                {0.25, 0.25, 0.25, 0.25},
                {-0.25/ WIDTH_LENGTH_AVERAGE, -0.25/ WIDTH_LENGTH_AVERAGE, 0.25/ WIDTH_LENGTH_AVERAGE, 0.25/ WIDTH_LENGTH_AVERAGE},
                {-0.25, 0.25, 0.25, -0.25}
        };
    }

    /**
     *  The initialize() method is called automatically when the robot's graphical UI is loaded from the
     *  arm_bot.fxml markup file. It should be used to set up parts of the graphical UI that will change
     *  as the robot operates
     */
    public void initialize(){

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
     *  Create the HardwareMap object
     */
    protected void createHardwareMap(){
        //Instantiate a new (empty) HardwareMap
        hardwareMap = new HardwareMap();

        //Motor type to use for the DC Motors
        motorType = MotorType.Neverest40;

        //Add the drive motors and arm motor using HardwareMap.put(...) method
        String[] motorNames = new String[] {"back_left_motor", "front_left_motor", "front_right_motor", "back_right_motor", "arm_motor"};
        for (String name: motorNames) hardwareMap.put(name, new DcMotorImpl(motorType));

        //Add the distance sensors
        String[] distNames = new String[]{"front_distance", "left_distance", "back_distance", "right_distance"};
        for (String name: distNames) hardwareMap.put(name, controller.new DistanceSensorImpl());

        //Add the BNO055IMUImpl sensor
        hardwareMap.put("imu", new BNO055IMUImpl(this, 10));

        //Add the ColorSensorImpl sensor
        hardwareMap.put("color_sensor", controller.new ColorSensorImpl());

        //Add the ServoImpl object
        hardwareMap.put("hand_servo", new ServoImpl());
    }

    /**
     * Update robot position on field and update the robot sensors
     * @param millis milliseconds since the previous update
     */
    public synchronized void updateStateAndSensors(double millis){

        // Array to contain the interval change in ticks for each drive motor.
        double[] deltaPos = new double[4];

        //Array to contain the interval distance travelled by each wheel (ticks * wheel_circumference/ticks_per_rotation)
        double[] w = new double[4];

        /*
        Assign values to the deltaPos and w array elements. Note that the update(millis) method of DcMotorImpl returns
        the interval number of ticks (as a floating point) since the preceeding call to update(millis).  Negate
        the values for the left-sided wheels.
         */
        for (int i = 0; i < 4; i++) {
            deltaPos[i] = motors[i].update(millis);
            w[i] = deltaPos[i] * WHEEL_CIRCUMFERENCE / motorType.TICKS_PER_ROTATION;
            if (i < 2) w[i] = -w[i];
        }

        /*
        The change of robot position is determined using matrix multiplication of the transform matrix (tWR)
        times the wheel motion array w. The elements in the resulting robotDeltaPos array are as follows:

                robotDeltaPos[0]: interval motion in the Robot-X direction (i.e., to the robot right or left)
                robotDeltaPos[1]: interval motion in the Robot-Y direction (i.e., robot forward or reverse)
                robotDeltaPos[2]: interval counter-clockwise rotation of robot in radians
         */
        double[] robotDeltaPos = new double[] {0,0,0,0};
        for (int i=0; i<4; i++){
            for (int j = 0; j<4; j++){
                robotDeltaPos[i] += tWR[i][j] * w[j];
            }
        }

        //Changes of position in ROBOT coordinate-space
        double dxR = robotDeltaPos[0];
        double dyR = robotDeltaPos[1];
        double headingChange = robotDeltaPos[2];

        /*
        During the interval, the AVERAGE heading of the robot is the heading at the beginning of the interval,
        plus one half of the heading change.
         */
        double avgHeading = headingRadians + headingChange / 2.0;

        /*
        Now convert from the changes of position in ROBOT coordinate-space to the new position of the robot
        in FIELD coordinate-space.
         */
        double sin = Math.sin(avgHeading);
        double cos = Math.cos(avgHeading);

        x += dxR * cos - dyR * sin;
        y += dxR * sin + dyR * cos;
        headingRadians += headingChange;

        /*
        This code is necessary to constrain x and y, so the robot can't escape the field.
         */
        if (x >  (halfFieldWidth - halfBotWidth)) x = halfFieldWidth - halfBotWidth;
        else if (x < (halfBotWidth - halfFieldWidth)) x = halfBotWidth - halfFieldWidth;
        if (y > (halfFieldWidth - halfBotWidth)) y = halfFieldWidth - halfBotWidth;
        else if (y < (halfBotWidth - halfFieldWidth)) y = halfBotWidth - halfFieldWidth;

        /*
        This code restrains the robot heading (in radians) to the -pi to +pi range (i.e., -180 to +180 degrees)
         */
        if (headingRadians > Math.PI) headingRadians -= 2.0 * Math.PI;
        else if (headingRadians < -Math.PI) headingRadians += 2.0 * Math.PI;

        /*
        Update the sensors so that when they are called from the op mode, they will return the correct values.
        Note that updating the distance sensors is complicated, because they are located on the four sides of the
        robot, so the position and orientation of each sensor depends on the position and orientation of the bot.
         */
        imu.updateHeadingRadians(headingRadians);

        colorSensor.updateColor(x, y);

        final double piOver2 = Math.PI / 2.0;
        for (int i = 0; i<4; i++){
            double sensorHeading = AngleUtils.normalizeRadians(headingRadians + i * piOver2);
            distanceSensors[i].updateDistance( x - halfBotWidth * Math.sin(sensorHeading),
                    y + halfBotWidth * Math.cos(sensorHeading), sensorHeading);
        }

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
        for (int i=0; i<4; i++) motors[i].stopAndReset();
        armMotor.stopAndReset();
        imu.close();
    }


}
