package virtual_robot.robots.classes;

import com.qualcomm.hardware.bosch.BNO055IMUImpl;
import com.qualcomm.hardware.bosch.BNO055IMUNew;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.hardware.configuration.MotorType;
import javafx.fxml.FXML;
import javafx.scene.shape.Rectangle;
import javafx.scene.transform.Rotate;
import org.dyn4j.dynamics.Body;
import org.dyn4j.geometry.MassType;
import org.dyn4j.geometry.Vector2;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import virtual_robot.config.Config;
import virtual_robot.controller.*;
import virtual_robot.util.AngleUtils;
import virtual_robot.util.Vector2D;

/**
 * For internal use only. Represents a robot with four swerve drives, color sensor, four distance sensors,
 * a BNO055IMU, and a Servo-controlled arm on the back.
 *
 * SwerveBot is the controller class for the "swerve_bot.fxml" markup file.
 *
 */
//@BotConfig(name = "Swerve Bot", filename = "swerve_bot")
public class SwerveBot extends VirtualBot {

    private final MotorType MOTOR_TYPE = MotorType.Neverest40;
    private DcMotorExImpl[] motors = null;
    private CRServoImpl[] crServos = null;
    private DeadWheelEncoder[] steerEncoders = null;
    private BNO055IMUImpl imu = null;
    private BNO055IMUNew imuNew = null;
    private VirtualRobotController.ColorSensorImpl colorSensor = null;
    private ServoImpl servo = null;
    private VirtualRobotController.DistanceSensorImpl[] distanceSensors = null;

    // backServoArm is instantiated during loading via a fx:id property.
    @FXML Rectangle backServoArm;

    //Rectangles representing the wheels, loaded via fx:id properties
    @FXML Rectangle rectFL;
    @FXML Rectangle rectFR;
    @FXML Rectangle rectBL;
    @FXML Rectangle rectBR;

    private double wheelRadius;
    private double interWheelWidth;
    private double interWheelLength;

    //Vectors representing positions of each wheel relative to robot center, in field units
    private final Vector2[] WHEEL_POS = new Vector2[4];

    //Maximum frictional force that can be applied to a single wheel by the floor
    private double maxWheelForce;


    public SwerveBot(){
        super();
    }

    public void initialize(){
        super.initialize();

        hardwareMap.setActive(true);
        motors = new DcMotorExImpl[]{
                (DcMotorExImpl)hardwareMap.get(DcMotorEx.class, "back_left_motor"),
                (DcMotorExImpl)hardwareMap.get(DcMotorEx.class, "front_left_motor"),
                (DcMotorExImpl)hardwareMap.get(DcMotorEx.class, "front_right_motor"),
                (DcMotorExImpl)hardwareMap.get(DcMotorEx.class, "back_right_motor")
        };
        crServos = new CRServoImpl[] {
                (CRServoImpl)hardwareMap.get(CRServoImpl.class, "back_left_crservo"),
                (CRServoImpl)hardwareMap.get(CRServoImpl.class, "front_left_crservo"),
                (CRServoImpl)hardwareMap.get(CRServoImpl.class, "front_right_crservo"),
                (CRServoImpl)hardwareMap.get(CRServoImpl.class, "back_right_crservo")
        };
        steerEncoders = new DeadWheelEncoder[] {
                (DeadWheelEncoder)hardwareMap.get(DeadWheelEncoder.class, "back_left_encoder"),
                (DeadWheelEncoder)hardwareMap.get(DeadWheelEncoder.class, "front_left_encoder"),
                (DeadWheelEncoder)hardwareMap.get(DeadWheelEncoder.class, "front_right_encoder"),
                (DeadWheelEncoder)hardwareMap.get(DeadWheelEncoder.class, "back_right_encoder")
        };
        distanceSensors = new VirtualRobotController.DistanceSensorImpl[]{
                hardwareMap.get(VirtualRobotController.DistanceSensorImpl.class, "front_distance"),
                hardwareMap.get(VirtualRobotController.DistanceSensorImpl.class, "left_distance"),
                hardwareMap.get(VirtualRobotController.DistanceSensorImpl.class, "back_distance"),
                hardwareMap.get(VirtualRobotController.DistanceSensorImpl.class, "right_distance")
        };
        imu = hardwareMap.get(BNO055IMUImpl.class, "imu");
        imuNew = hardwareMap.get(BNO055IMUNew.class, "imu");
        colorSensor = (VirtualRobotController.ColorSensorImpl)hardwareMap.colorSensor.get("color_sensor");

        servo = (ServoImpl)hardwareMap.servo.get("back_servo");

        wheelRadius = 0.5 * botWidth / (4.5 * VirtualField.PIXELS_PER_METER);

        interWheelWidth = botWidth * 8.0 / (9.0 * VirtualField.PIXELS_PER_METER);
        interWheelLength = botWidth * 7.0 / (9.0 * VirtualField.PIXELS_PER_METER);

        WHEEL_POS[0] = new Vector2(-interWheelWidth/2, -interWheelLength/2);
        WHEEL_POS[1] = new Vector2(-interWheelWidth/2, interWheelLength/2);
        WHEEL_POS[2] = new Vector2(interWheelWidth/2, interWheelLength/2);
        WHEEL_POS[3] = new Vector2(interWheelWidth/2, -interWheelLength/2);

        maxWheelForce = Config.FIELD_FRICTION_COEFF * 9.8 * chassisBody.getMass().getMass() / 4.0;

        hardwareMap.setActive(false);

        backServoArm.getTransforms().add(new Rotate(0, 37.5, 67.5));
    }

    protected void createHardwareMap(){
        hardwareMap = new HardwareMap();
        String[] motorNames = new String[] {"back_left_motor", "front_left_motor", "front_right_motor", "back_right_motor"};
        String[] encoderNames = new String[] {"back_left_encoder", "front_left_encoder", "front_right_encoder", "back_right_encoder"};
        String[] crservoNames = new String[] {"back_left_crservo", "front_left_crservo", "front_right_crservo", "back_right_crservo"};
        for (String name: motorNames) hardwareMap.put(name, new DcMotorExImpl(MOTOR_TYPE));
        for (String name: crservoNames) hardwareMap.put(name, new CRServoImpl(360));
        for (String name: encoderNames) hardwareMap.put(name, new DeadWheelEncoder(MOTOR_TYPE));
        String[] distNames = new String[]{"front_distance", "left_distance", "back_distance", "right_distance"};
        for (String name: distNames) hardwareMap.put(name, controller.new DistanceSensorImpl());
        hardwareMap.put("imu", new BNO055IMUImpl(this, 10));
        hardwareMap.put("imu", new BNO055IMUNew(this, 10));
        hardwareMap.put("color_sensor", controller.new ColorSensorImpl());
        hardwareMap.put("back_servo", new ServoImpl());
    }

    public synchronized void updateStateAndSensors(double millis){

        x = chassisBody.getTransform().getTranslationX() * VirtualField.PIXELS_PER_METER;
        y = chassisBody.getTransform().getTranslationY() * VirtualField.PIXELS_PER_METER;
        headingRadians = chassisBody.getTransform().getRotationAngle();

        Vector2 velocity = chassisBody.getLinearVelocity();
        double omega = chassisBody.getAngularVelocity();

        /*
         *  This is not a realistic physics-based simulation. The virtual_robot simulator is kinematic. For the swerve bot,
         *  pure kinematic simulation isn't feasible, since the wheels can be aligned in such a way that lots of
         *  skidding is inevitable. This simulation ASSUMES that there is skidding, and assigns frictional forces to
         *  each wheel. Those forces depend on the speed at which the surface of that wheel is skidding relative
         *  to the floor (not true of real kinetic friction).
         */

        final double FORCE_COEFF = 4;         //in botMass/sec    i.e., (botMass*botWidth/sec^2)/(botWidth/sec)

        Vector2 totalForce = new Vector2(0,0);
        double totalTorque = 0;

        for (int i = 0; i < 4; i++) {
            motors[i].update(millis);
            double wSpd = motors[i].getVelocity(AngleUnit.RADIANS) * wheelRadius;
            boolean mtRev = MOTOR_TYPE.REVERSED;
            boolean dirRev = motors[i].getDirection() == DcMotorSimple.Direction.REVERSE;
            if (i<2 && (mtRev && dirRev || !mtRev && !dirRev) || i>=2 && (mtRev && !dirRev || !mtRev && dirRev)) {
                wSpd = -wSpd;
            }
            steerEncoders[i].update(Math.toRadians(crServos[i].updatePositionDegrees(millis)), millis);
            double steer = Math.toRadians(crServos[i].getPositionDegrees());
            Vector2 w = new Vector2(-wSpd * Math.sin(steer+headingRadians), wSpd * Math.cos(steer+headingRadians));
            Vector2 v = velocity.sum(new Vector2(WHEEL_POS[i]).rotate(headingRadians + Math.PI/2).product(omega));
            Vector2 skidVelocity = v.difference(w);       //skid velocity in meters/sec
            //frictional force on wheel in kg*meters/sec^2
            Vector2 wheelForce = skidVelocity.product(-FORCE_COEFF * chassisBody.getMass().getMass());
            if (wheelForce.getMagnitude() > maxWheelForce) wheelForce.setMagnitude(maxWheelForce);
            totalForce.add(wheelForce);
            Vector2 torqueArm = new Vector2(WHEEL_POS[i]).rotate(headingRadians);
            double torque = torqueArm.cross(wheelForce);
            totalTorque += torque;                         //in kg*m^2/sec^2
        }

        chassisBody.applyForce(totalForce);
        chassisBody.applyTorque(totalTorque);

        imu.updateHeadingRadians(headingRadians);
        imuNew.updateHeadingRadians(headingRadians);

        colorSensor.updateColor(x, y);

        final double piOver2 = Math.PI / 2.0;

        for (int i = 0; i<4; i++){
            double sensorHeading = AngleUtils.normalizeRadians(headingRadians + i * piOver2);
            distanceSensors[i].updateDistance( x - halfBotWidth * Math.sin(sensorHeading),
                    y + halfBotWidth * Math.cos(sensorHeading), sensorHeading);
        }

    }

    public synchronized void updateDisplay(){
        super.updateDisplay();
        ((Rotate)backServoArm.getTransforms().get(0)).setAngle(-180.0 * servo.getInternalPosition());
        rectBL.setRotate(-crServos[0].getPositionDegrees());
        rectFL.setRotate(-crServos[1].getPositionDegrees());
        rectFR.setRotate(-crServos[2].getPositionDegrees());
        rectBR.setRotate(-crServos[3].getPositionDegrees());
    }

    public void powerDownAndReset(){
        for (int i=0; i<4; i++) {
            motors[i].stopAndReset();
            crServos[i].setPower(0);
        }
        imu.close();
        chassisBody.setAngularVelocity(0);
        chassisBody.setLinearVelocity(0,0);
    }

    /**
     *  Set up the chassisBody and add it to the dyn4j world. This method creates a Body and adds a BodyFixture
     *  containing a Rectangle. Add the chassis body to the world.
     *     Large mass: collisions with lightweight game elements will have neglible effect on bot motion
     *     The "friction" of 0 refers to robot-game element and robot-wall friction (NOT robot-floor)
     *     The "restitution" of 0 refers to "bounce" when robot collides with wall and game elements
     *
     *     May want to change density, friction, and restitution to obtain desired behavior
     *
     *  The filter set on the chassisFixture indicates what other things the robot is capable of colliding with
     */
    public void setUpChassisBody(){
        chassisBody = new Body();
        chassisBody.setUserData(this);
        double botWidthMeters = botWidth / VirtualField.PIXELS_PER_METER;
        chassisFixture = chassisBody.addFixture(
                new org.dyn4j.geometry.Rectangle(botWidthMeters, botWidthMeters), 71.76, 0, 0);
        chassisRectangle = (org.dyn4j.geometry.Rectangle)chassisFixture.getShape();
        chassisFixture.setFilter(Filters.CHASSIS_FILTER);
        chassisBody.setMass(MassType.NORMAL);
        world.addBody(chassisBody);
    }

}
