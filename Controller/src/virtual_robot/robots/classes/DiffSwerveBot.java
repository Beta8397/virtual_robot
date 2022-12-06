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
import virtual_robot.config.Config;
import virtual_robot.controller.*;
import virtual_robot.util.AngleUtils;

/**
 * For internal use only. Represents a robot with two differential swerve units, color sensor, four distance sensors,
 * a BNO055IMU, and a Servo-controlled arm on the back.
 *
 * DiffSwerveBot is the controller class for the "diff_swerve_bot.fxml" markup file.
 *
 */
//@BotConfig(name = "Differential Swerve Bot", filename = "diff_swerve_bot")
public class DiffSwerveBot extends VirtualBot {

    private final MotorType MOTOR_TYPE = MotorType.NeverestOrbital20;
    private DcMotorExImpl[] motors = null;
    private BNO055IMUImpl imu = null;
    private BNO055IMUNew imuNew = null;
    private VirtualRobotController.ColorSensorImpl colorSensor = null;
    private ServoImpl servo = null;
    private VirtualRobotController.DistanceSensorImpl[] distanceSensors = null;

    // backServoArm is instantiated during loading via a fx:id property.
    @FXML Rectangle backServoArm;

    //Rectangles representing the wheels, loaded via fx:id properties
    @FXML Rectangle rectLeft;
    @FXML Rectangle rectRight;

    private double wheelCircumferenceMeters;
    private double interWheelWidth;

    //Vectors representing positions of each wheel relative to robot center, in field units
    private final Vector2[] WHEEL_POS = new Vector2[2];

    //Scalars representing the STEER, in radians of the left and right motors
    private double[] steer = new double[2];

    private final double STEER_RATIO = 4;
    private final double DRIVE_RATIO = 1.25;

    private double maxWheelForce;

    public DiffSwerveBot(){
        super();
    }

    public void initialize() {
        super.initialize();
        hardwareMap.setActive(true);
        motors = new DcMotorExImpl[]{
                hardwareMap.get(DcMotorExImpl.class, "bottom_left_motor"),
                hardwareMap.get(DcMotorExImpl.class, "top_left_motor"),
                hardwareMap.get(DcMotorExImpl.class, "bottom_right_motor"),
                hardwareMap.get(DcMotorExImpl.class, "top_right_motor")
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
        wheelCircumferenceMeters = Math.PI * botWidth / (4.5 * VirtualField.PIXELS_PER_METER);
        interWheelWidth = botWidth * 12.0 / (18.0 * VirtualField.PIXELS_PER_METER);

        WHEEL_POS[0] = new Vector2(-interWheelWidth/2, 0);         //Left
        WHEEL_POS[1] = new Vector2(interWheelWidth/2, 0);          //Right

        maxWheelForce = Config.FIELD_FRICTION_COEFF * 9.8 * chassisBody.getMass().getMass() / 2.0;

        hardwareMap.setActive(false);
        backServoArm.getTransforms().add(new Rotate(0, 37.5, 67.5));
    }

    protected void createHardwareMap(){
        hardwareMap = new HardwareMap();
        String[] motorNames = new String[] {"bottom_left_motor", "top_left_motor", "bottom_right_motor", "top_right_motor"};
        for (String name: motorNames) hardwareMap.put(name, new DcMotorExImpl(MOTOR_TYPE));
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

        final double FORCE_COEFF = 8;         //in botMass/sec    i.e., (botMass*botWidth/sec^2)/(botWidth/sec)

        Vector2 totalForce = new Vector2(0,0);
        double totalTorque = 0;

        for (int i = 0; i < 2; i++) {
            motors[2*i].update(millis);
            motors[2*i+1].update(millis);

            boolean bottomRev = motors[2*i].getDirection() == DcMotorSimple.Direction.REVERSE;
            boolean topRev = motors[2*i+1].getDirection() == DcMotorSimple.Direction.REVERSE;
            double bottomSpd = motors[2 * i].getVelocity();
            if (i == 0 && (MOTOR_TYPE.REVERSED && !bottomRev || !MOTOR_TYPE.REVERSED && bottomRev)
                    || i == 1 && (MOTOR_TYPE.REVERSED && bottomRev || !MOTOR_TYPE.REVERSED && !bottomRev)) {
                bottomSpd = -bottomSpd;
            }
            double topSpd = motors[2*i+1].getVelocity();
            if (i == 0 && (MOTOR_TYPE.REVERSED && !topRev || !MOTOR_TYPE.REVERSED && topRev)
                    || i == 1 && (MOTOR_TYPE.REVERSED && topRev || !MOTOR_TYPE.REVERSED && !topRev)) {
                topSpd = -topSpd;
            }
            double driveSpd = 0.5 * (bottomSpd - topSpd)
                    * wheelCircumferenceMeters / (DRIVE_RATIO * MOTOR_TYPE.TICKS_PER_ROTATION);
            steer[i] = -Math.PI * (motors[2*i].getActualPosition() + motors[2*i+1].getActualPosition())
                    / (STEER_RATIO * MOTOR_TYPE.TICKS_PER_ROTATION);
            Vector2 w = new Vector2(-driveSpd * Math.sin(steer[i]+headingRadians), driveSpd * Math.cos(steer[i]+headingRadians));
            Vector2 v = velocity.sum((new Vector2(WHEEL_POS[i]).rotate(headingRadians+Math.PI/2).product(omega)));
            Vector2 skidVelocity = v.difference(w);       //skid velocity in fieldUnit/sec
            Vector2 wheelForce = skidVelocity.product(-FORCE_COEFF * chassisBody.getMass().getMass());          //frictional force on wheel in kg*m/sec^2
            if (wheelForce.getMagnitude() > maxWheelForce){
                wheelForce.setMagnitude(maxWheelForce);
            }
            totalForce.add(wheelForce);
            Vector2 torqueArm = new Vector2(WHEEL_POS[i]).rotate(headingRadians);
            double torque = torqueArm.cross(wheelForce);
            totalTorque += torque;
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
        rectLeft.setRotate(-Math.toDegrees(steer[0]));
        rectRight.setRotate(-Math.toDegrees(steer[1]));
    }

    public void powerDownAndReset(){
        for (int i=0; i<2; i++) {
            motors[2*i].stopAndReset();
            motors[2*i+1].stopAndReset();
            steer[i] = 0;
        }
        updateDisplay();
        imu.close();
        chassisBody.setLinearVelocity(0,0);
        chassisBody.setAngularVelocity(0);
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
