package virtual_robot.robots.classes;

import com.qualcomm.hardware.bosch.BNO055IMUImpl;
import com.qualcomm.hardware.bosch.BNO055IMUNew;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorExImpl;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.MotorType;
import org.dyn4j.dynamics.Body;
import org.dyn4j.geometry.MassType;
import org.dyn4j.geometry.Vector2;
import org.firstinspires.ftc.robotcore.external.matrices.GeneralMatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.MatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import virtual_robot.config.Config;
import virtual_robot.controller.Filters;
import virtual_robot.controller.VirtualBot;
import virtual_robot.controller.VirtualField;
import virtual_robot.controller.VirtualRobotController;
import virtual_robot.util.AngleUtils;

/**
 * For internal use only. Base class for a Kiwi drive bot with three Omniwheels, color sensor,
 * four distance sensors, and a BNO055IMU.
 */
public abstract class KiwiPhysicsBase extends VirtualBot {

    public final MotorType MOTOR_TYPE;
    private DcMotorExImpl[] motors = null;
    private BNO055IMUImpl imu = null;
    BNO055IMUNew imuNew = null;
    private VirtualRobotController.ColorSensorImpl colorSensor = null;
    private VirtualRobotController.DistanceSensorImpl[] distanceSensors = null;

    /*
     * Gear ratio for any external gears added in drive train. For now, this is just 1.0. Could easily
     * add a constructor to MecanumPhysicsBase that allows this to be set to some other value.
     */
    protected double gearRatioWheel = 1.0;

    /*
     * Robot geometry, in pixels. These will be calculated in the initialize() method. They cannot be computed
     * here, because botwidth is not known until the time of construction.
     */
    private double wheelCircumference = 4 * Math.PI; // inches
    private double baseRadius = 8;  // inches

    /*
     * Transform from wheel motion to robot motion (KINETIC MODEL). This will be computed in the initialize()
     * method, after basic robot geometry is computed.
     */
    private double[][] tWR;

    GeneralMatrixF M_ForceWheelToRobot; // Converts from individual wheel forces to total force/torque on robot
    MatrixF M_ForceRobotToWheel;  // Converts from total force/torque on robot to individual wheel forces
    protected float maxWheelForce; // Need to assign this in the initialize method.

    /**
     * No-param constructor. Uses the default motor type of Neverest 40
     */
    public KiwiPhysicsBase() {
        super();
        MOTOR_TYPE = MotorType.Neverest40;
    }

    public KiwiPhysicsBase(MotorType driveMotorType){
        super();
        MOTOR_TYPE = driveMotorType;
    }

    public KiwiPhysicsBase(MotorType driveMotorType, double baseRadius){
        this(driveMotorType);
        this.baseRadius = baseRadius;
    }

    public void initialize(){
        super.initialize();
        hardwareMap.setActive(true);
        motors = new DcMotorExImpl[]{
                (DcMotorExImpl) hardwareMap.get(DcMotorEx.class, "front_motor"),
                (DcMotorExImpl) hardwareMap.get(DcMotorEx.class, "back_left_motor"),
                (DcMotorExImpl) hardwareMap.get(DcMotorEx.class, "back_right_motor")
        };
        distanceSensors = new VirtualRobotController.DistanceSensorImpl[]{
                hardwareMap.get(VirtualRobotController.DistanceSensorImpl.class, "front_distance"),
                hardwareMap.get(VirtualRobotController.DistanceSensorImpl.class, "left_distance"),
                hardwareMap.get(VirtualRobotController.DistanceSensorImpl.class, "back_distance"),
                hardwareMap.get(VirtualRobotController.DistanceSensorImpl.class, "right_distance")
        };
        imu = hardwareMap.get(BNO055IMUImpl.class, "imu");
        imuNew = hardwareMap.get(BNO055IMUNew.class, "imu");
        colorSensor = (VirtualRobotController.ColorSensorImpl) hardwareMap.colorSensor.get("color_sensor");



        tWR = new double[][]{
                {-2.0/3.0, 1.0/3.0, 1.0/3.0},
                {0, -1.0/Math.sqrt(3), 1.0/Math.sqrt(3)},
                {1.0/(3.0*baseRadius), 1.0/(3.0*baseRadius), 1.0/(3.0*baseRadius)}
        };

        /*
         * Converts from the frictional forces at each wheel to the X, Y forces, and Torque
         * forces on the robot. (in the ROBOT coordinate system)
         */
        M_ForceWheelToRobot = new GeneralMatrixF(3, 3, new float[]{
                -1, 0.5f, 0.5f,
                0, -(float)Math.sqrt(3)/2f, (float)Math.sqrt(3)/2f,
                (float)baseRadius, (float)baseRadius, (float)baseRadius});

        /*
         * Converts from X & Y Forces, Torque, and "Fail" force on robot to the corresponding forces
         * at each wheel. (in the ROBOT coordinate system)
         */
        M_ForceRobotToWheel = M_ForceWheelToRobot.inverted();

        // Maximum possible frictional force (in robot-X direction) between field and any individual robot wheel.
        // Note the division by 4 (assumes each wheel gets 1/4 of robot mass) and the division by sqrt(2) (because
        // the X-direction force is 1/sqrt(2) times the total friction force on the wheel.
        maxWheelForce = (float)(9.8 * chassisBody.getMass().getMass()
                * Config.FIELD_FRICTION_COEFF / 3.0);

        hardwareMap.setActive(false);

    }

    /**
     * Create the hardware map for this robot. This will include the drive motors, distance sensors, BNO055IMU,
     * and color sensor. Child classes can override this method to add additional hardware. In that case,
     * the first statement in the override method should be: super.createHardwareMap().
     */
    protected void createHardwareMap() {
        hardwareMap = new HardwareMap();
        String[] motorNames = new String[]{"front_motor", "back_left_motor", "back_right_motor"};
        for (int i=0; i<3; i++){
            hardwareMap.put(motorNames[i], new DcMotorExImpl(MOTOR_TYPE, motorController0, i));
        }
        String[] distNames = new String[]{"front_distance", "left_distance", "back_distance", "right_distance"};
        for (String name : distNames) hardwareMap.put(name, controller.new DistanceSensorImpl());
        hardwareMap.put("imu", new BNO055IMUImpl(this, 10));
        hardwareMap.put("imu", new BNO055IMUNew(this, 10));
        hardwareMap.put("color_sensor", controller.new ColorSensorImpl());
    }

    /**
     * Update the position of the robot on the field, as well as the distance, BNO055IMU, and color sensors.
     *
     * Updating robot position involves:
     *     1) Get new position and orientation from the dyn4j physics engine.
     *     2) Using kinetic model, "preload" the chassis body with force and torque to be applied
     *        during the next update of the physics engine.
     *
     * @param millis milliseconds since the previous update
     */
    public synchronized void updateStateAndSensors(double millis) {

        /*
         * Get updated position and heading from the dyn4j body (chassisBody)
         */
        x = chassisBody.getTransform().getTranslationX() * VirtualField.PIXELS_PER_METER;
        y = chassisBody.getTransform().getTranslationY() * VirtualField.PIXELS_PER_METER;
        headingRadians = chassisBody.getTransform().getRotationAngle();

        // Compute new wheel speeds in pixel units per second

        double[] wSpd = new double[3];
        for (int i=0; i<3; i++){
            motors[i].update(millis);
            wSpd[i] = motors[i].getVelocity(AngleUnit.RADIANS) * gearRatioWheel * wheelCircumference  / (2.0 * Math.PI);
            boolean mtRev = MOTOR_TYPE.REVERSED;
            boolean dirRev = motors[i].getDirection() == DcMotorSimple.Direction.REVERSE;
            if ( (mtRev && !dirRev) || (!mtRev && dirRev) ) wSpd[i] = -wSpd[i];
        }

        /*
         * Based on wheel speeds, compute the target final robot velocity and angular speed, in the robot
         * coordinate system.
         */
        double[] robotTargetSpd = new double[]{0,0,0};
        for (int i=0; i<3; i++){
            for (int j=0; j<3; j++){
                robotTargetSpd[i] += tWR[i][j] * wSpd[j];
            }
        }

        robotTargetSpd[0] /= VirtualField.INCHES_PER_METER;
        robotTargetSpd[1] /= VirtualField.INCHES_PER_METER;

        /*
         * Compute an estimated final heading. This is used only to convert the target final robot velocity
         * from robot coordinate system to world coordinate system.
         */
        double t = millis / 1000.0;
        double estFinalHeading = headingRadians + 0.5 * (chassisBody.getAngularVelocity() + robotTargetSpd[2]) * t;
        double sinFinal = Math.sin(estFinalHeading);
        double cosFinal = Math.cos(estFinalHeading);

        /*
         * Convert target final robot velocity from robot coordinate system to world coordinate system.
         */
        Vector2 estFinalVelocity = new Vector2(
                robotTargetSpd[0]*cosFinal - robotTargetSpd[1]*sinFinal,
                robotTargetSpd[0]*sinFinal + robotTargetSpd[1]*cosFinal
        );

        /*
         * Compute the force and torque that would be required to achieve the target changes in robot velocity and
         * angular speed (F = ma,  tau = I*alpha)
         */
        Vector2 force = estFinalVelocity.difference(chassisBody.getLinearVelocity()).product(chassisBody.getMass().getMass()/t);
        double torque = (robotTargetSpd[2] - chassisBody.getAngularVelocity()) * chassisBody.getMass().getInertia()/t;

        /*
         * Convert the proposed force from world to robot coordinate system.
         */
        double sinHd = Math.sin(headingRadians);
        double cosHd = Math.cos(headingRadians);

        float fXR = (float)(force.x*cosHd + force.y*sinHd);
        float fYR = (float)(-force.x*sinHd + force.y*cosHd);

        // VectorF containing total force and torque required on bot to achieve the tentative position change,
        // in robot coordinate system
        VectorF totalForce = new VectorF(fXR, fYR, (float)torque);

        // Required friction force from floor to achieve the tentative position change. For now, this will
        // be the same as totalForce, but may want to add offsets to collision forces
        VectorF frictionForces = new VectorF(totalForce.get(0), totalForce.get(1), totalForce.get(2));

        // Determine the X-direction forces that would be required on each of the bot's four wheels to achieve
        // the total frictional force and torque predicted by the kinematic model. Note that the magnitude of
        // total force on each wheel is sqrt(2) times abs(x-direction force). (ROBOT coordinate system)

        VectorF wheel_Forces = M_ForceRobotToWheel.multiplied(frictionForces);

        //If any of the wheel forces exceeds the product of muStatic*mass*gravity, reduce the magnitude
        //of that force to muKinetic*mass*gravity, keeping the direction the same

        for (int i=0; i<3; i++){
            float f = wheel_Forces.get(i);
            if (Math.abs(f) > maxWheelForce) {
                wheel_Forces.put(i, maxWheelForce * Math.signum(f));
            }
        }

        //Based on the adjusted forces at each wheel, determine net frictional force and torque on the bot,
        //Force is in ROBOT COORDINATE system

        frictionForces = M_ForceWheelToRobot.multiplied(wheel_Forces);

        // Convert these adjusted friction forces to WORLD COORDINATES  and put into the original
        // force and torque variables
        force = new Vector2(frictionForces.get(0)*cosHd - frictionForces.get(1)*sinHd,
                frictionForces.get(0)*sinHd + frictionForces.get(1)*cosHd);
        torque = frictionForces.get(2);

        /*
         * Apply the adjusted frictional force and torque to the chassisBody
         *
         * Note:  We are only applying the frictional forces from the floor, NOT the collision forces. The
         *        collision forces will be applied automatically during the next update of the world by the
         *        dyn4j physics engine.
         */

        chassisBody.applyForce(force);
        chassisBody.applyTorque(torque);

        /*
         * Update the sensors
         */
        imu.updateHeadingRadians(headingRadians);
        imuNew.updateHeadingRadians(headingRadians);

        colorSensor.updateColor(x, y);

        final double piOver2 = Math.PI / 2.0;

        for (int i = 0; i < 4; i++) {
            double sensorHeading = AngleUtils.normalizeRadians(headingRadians + i * piOver2);
            distanceSensors[i].updateDistance(x - baseRadius * VirtualField.PIXELS_PER_INCH * Math.sin(sensorHeading),
                    y + baseRadius * VirtualField.PIXELS_PER_INCH * Math.cos(sensorHeading), sensorHeading);
        }

    }

    /**
     * Display the robot in the current orientation and position.
     */
    public synchronized void updateDisplay() {
        super.updateDisplay();
    }

    public void powerDownAndReset() {
        for (int i = 0; i < 3; i++) motors[i].stopAndReset();
        imu.close();
        chassisBody.setAngularVelocity(0);
        chassisBody.setLinearVelocity(0,0);
    }

    /**
     *  Set up the chassisBody and add it to the dyn4j world. This method creates a Body and adds a BodyFixture
     *  containing a Rectangle. Add the chassis body to the world.
     *     The density value of 71.76 kg/m2 results in chassis mass of 15 kg.
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
        double botRadiusMeters = baseRadius / VirtualField.INCHES_PER_METER;
        chassisFixture = chassisBody.addFixture(
                new org.dyn4j.geometry.Circle(botRadiusMeters), 71.76, 0, 0);
//        chassisRectangle = (org.dyn4j.geometry.Rectangle)chassisFixture.getShape();
        chassisFixture.setFilter(Filters.CHASSIS_FILTER);
        chassisBody.setMass(MassType.NORMAL);
        world.addBody(chassisBody);
    }


    /**
     * Constrain robot to the boundaries X_MIN, X_MAX, Y_MIN, Y_MAX
     * If physics simulation is being used, this should not be called during active simulation (just let robot collide
     * with walls). But, it is needed for positioning robot with mouse.
     */
    protected void constrainToBoundaries(){
        final double botRadiusPixels = baseRadius * VirtualField.PIXELS_PER_INCH;
        if (Math.abs(x) >  (VirtualField.X_MAX - botRadiusPixels)) {
            x = (VirtualField.X_MAX - botRadiusPixels) * Math.signum(x);
        }
        if (Math.abs(y) > (VirtualField.Y_MAX - botRadiusPixels)) {
            y = (VirtualField.Y_MAX - botRadiusPixels) * Math.signum(y);
        }
    }

}
