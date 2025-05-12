package virtual_robot.robots.classes;

import com.qualcomm.hardware.CommonOdometry;
import com.qualcomm.hardware.bosch.BNO055IMUImpl;
import com.qualcomm.hardware.bosch.BNO055IMUNew;
import com.qualcomm.hardware.digitalchickenlabs.OctoQuadImpl;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriverInternal;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.hardware.sparkfun.SparkFunOTOSInternal;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorExImpl;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.MotorType;
import javafx.scene.input.MouseEvent;
import org.dyn4j.dynamics.Body;
import org.dyn4j.geometry.MassType;
import org.dyn4j.geometry.Vector2;
import org.firstinspires.ftc.robotcore.external.matrices.GeneralMatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.MatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import virtual_robot.config.Config;
import virtual_robot.controller.*;
import virtual_robot.util.AngleUtils;

/**
 * For internal use only. Base class for a physics-based robot with four mecanum wheels, color sensor,
 * four distance sensors, and a BNO055IMU.
 */
public abstract class MecanumPhysicsBase extends VirtualBot {

    public final MotorType MOTOR_TYPE;
    private DcMotorExImpl[] motors = null;
    private BNO055IMUImpl imu = null;
    BNO055IMUNew imuNew = null;
    private CommonOdometry odo = new CommonOdometry();
    private SparkFunOTOSInternal sparkFunOTOSInternal = null;
    private GoBildaPinpointDriverInternal goBildaPinpointDriverInternal = null;
    private VirtualRobotController.ColorSensorImpl colorSensor = null;
    private VirtualRobotController.DistanceSensorImpl[] distanceSensors = null;
    protected OctoQuadImpl octoQuad = null;

    /*
     * Gear ratio for any external gears added in drive train. For now, this is just 1.0. Could easily
     * add a constructor to MecanumPhysicsBase that allows this to be set to some other value.
     */
    protected double gearRatioWheel = 1.0;

    /*
     * Robot geometry, in pixels. These will be calculated in the initialize() method. They cannot be computed
     * here, because botwidth is not known until the time of construction.
     */
    private double wheelCircumference;
    private double interWheelWidth;
    private double interWheelLength;
    private double wlAverage;

    /*
     * Transform from wheel motion to robot motion (KINETIC MODEL). This will be computed in the initialize()
     * method, after basic robot geometry is computed.
     */
    private double[][] tWR;

    GeneralMatrixF M_ForceWheelToRobot; // Converts from individual wheel forces to total force/torque on robot
    MatrixF M_ForceRobotToWheel;  // Converts from total force/torque on robot to individual wheel forces
    protected float maxWheelXForce; // Need to assign this in the initialize method.

    /**
     * No-param constructor. Uses the default motor type of Neverest 40
     */
    public MecanumPhysicsBase() {
        super();
        MOTOR_TYPE = MotorType.Neverest40;
    }

    public MecanumPhysicsBase(MotorType driveMotorType){
        super();
        MOTOR_TYPE = driveMotorType;
    }

    public void initialize(){
        super.initialize();
        hardwareMap.setActive(true);
        motors = new DcMotorExImpl[]{
                (DcMotorExImpl) hardwareMap.get(DcMotorEx.class, "back_left_motor"),
                (DcMotorExImpl) hardwareMap.get(DcMotorEx.class, "front_left_motor"),
                (DcMotorExImpl) hardwareMap.get(DcMotorEx.class, "front_right_motor"),
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

        sparkFunOTOSInternal = hardwareMap.get(SparkFunOTOSInternal.class, "sensor_otos");
        goBildaPinpointDriverInternal = hardwareMap.get(GoBildaPinpointDriverInternal.class, "pinpoint");

        octoQuad = hardwareMap.get(OctoQuadImpl.class, "octoquad");
        for (int i=0; i<4; i++){
            octoQuad.setEncoder(i, motors[i]);
        }

        wheelCircumference = Math.PI * botWidth / 4.5;
        interWheelWidth = botWidth * 8.0 / 9.0;
        interWheelLength = botWidth * 7.0 / 9.0;
        wlAverage = (interWheelLength + interWheelWidth) / 2.0;

        tWR = new double[][]{
                {-0.25, 0.25, -0.25, 0.25},
                {0.25, 0.25, 0.25, 0.25},
                {-0.25 / wlAverage, -0.25 / wlAverage, 0.25 / wlAverage, 0.25 / wlAverage},
                {-0.25, 0.25, 0.25, -0.25}
        };

        float RRt2 = 0.5f * (float)Math.sqrt(interWheelLength*interWheelLength + interWheelWidth*interWheelWidth)
                * (float)Math.sqrt(2.0) / (float)VirtualField.PIXELS_PER_METER;

        /*
         * Converts from the frictional X-component forces at each wheel to the X, Y forces, Torque, and "Fail"
         * forces on the robot. (in the ROBOT coordinate system)
         */
        M_ForceWheelToRobot = new GeneralMatrixF(4, 4, new float[]{
                1, 1, 1, 1,
                -1, 1, -1, 1,
                RRt2, -RRt2, -RRt2, RRt2,
                1, 1, -1, -1});

        /*
         * Converts from X & Y Forces, Torque, and "Fail" force on robot to the corresponding forces (X-component)
         * at each wheel. (in the ROBOT coordinate system)
         */
        M_ForceRobotToWheel = M_ForceWheelToRobot.inverted();

        // Maximum possible frictional force (in robot-X direction) between field and any individual robot wheel.
        // Note the division by 4 (assumes each wheel gets 1/4 of robot mass) and the division by sqrt(2) (because
        // the X-direction force is 1/sqrt(2) times the total friction force on the wheel.
        maxWheelXForce = (float)(9.8 * chassisBody.getMass().getMass()
                * Config.FIELD_FRICTION_COEFF / (4.0 * Math.sqrt(2)));

        hardwareMap.setActive(false);

    }

    /**
     * Create the hardware map for this robot. This will include the drive motors, distance sensors, BNO055IMU,
     * and color sensor. Child classes can override this method to add additional hardware. In that case,
     * the first statement in the override method should be: super.createHardwareMap().
     */
    protected void createHardwareMap() {
        hardwareMap = new HardwareMap();
        String[] motorNames = new String[]{"back_left_motor", "front_left_motor", "front_right_motor", "back_right_motor"};
        for (int i=0; i<4; i++){
            hardwareMap.put(motorNames[i], new DcMotorExImpl(MOTOR_TYPE, motorController0, i));
        }
        String[] distNames = new String[]{"front_distance", "left_distance", "back_distance", "right_distance"};
        for (String name : distNames) hardwareMap.put(name, controller.new DistanceSensorImpl());
        hardwareMap.put("imu", new BNO055IMUImpl(this, 10));
        hardwareMap.put("imu", new BNO055IMUNew(this, 10));
        hardwareMap.put("color_sensor", controller.new ColorSensorImpl());
        hardwareMap.put("sensor_otos", new SparkFunOTOSInternal(odo));
        hardwareMap.put("pinpoint", new GoBildaPinpointDriverInternal(odo));
        hardwareMap.put("octoquad", new OctoQuadImpl());
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
        double xMeters = chassisBody.getTransform().getTranslationX();
        double yMeters = chassisBody.getTransform().getTranslationY();
        x = xMeters * VirtualField.PIXELS_PER_METER;
        y = yMeters * VirtualField.PIXELS_PER_METER;
        headingRadians = chassisBody.getTransform().getRotationAngle();

        /*
         * Get updated velocities (for use by the SparkFunOTOS sensor)
         */
        Vector2 velocityMetersPerSec = chassisBody.getLinearVelocity();
        double vxMetersPerSec = velocityMetersPerSec.x;
        double vyMetersPerSec = velocityMetersPerSec.y;
        double angularVelocityRadiansPerSec = chassisBody.getAngularVelocity();

        // Compute new wheel speeds in pixel units per second

        double[] wSpd = new double[4];
        for (int i=0; i<4; i++){
            motors[i].update(millis);
            wSpd[i] = motors[i].getVelocity(AngleUnit.RADIANS) * gearRatioWheel * wheelCircumference  / (2.0 * Math.PI);
            boolean mtRev = MOTOR_TYPE.REVERSED;
            boolean dirRev = motors[i].getDirection() == DcMotorSimple.Direction.REVERSE;
            if (
                    i<2 && (mtRev && dirRev || !mtRev && !dirRev) || i>=2 && (mtRev && !dirRev || !mtRev && dirRev)
            ) wSpd[i] = -wSpd[i];
        }

        /*
         * Based on wheel speeds, compute the target final robot velocity and angular speed, in the robot
         * coordinate system.
         */
        double[] robotTargetSpd = new double[]{0,0,0,0};
        for (int i=0; i<4; i++){
            for (int j=0; j<4; j++){
                robotTargetSpd[i] += tWR[i][j] * wSpd[j];
            }
        }


        robotTargetSpd[0] /= VirtualField.PIXELS_PER_METER;
        robotTargetSpd[1] /= VirtualField.PIXELS_PER_METER;

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
        VectorF totalForce = new VectorF(fXR, fYR, (float)torque, 0);


        // Required friction force from floor to achieve the tentative position change. For now, this will
        // be the same as totalForce, but may want to add offsets to collision forces
        VectorF frictionForces = new VectorF(totalForce.get(0), totalForce.get(1), totalForce.get(2), totalForce.get(3));

        // Determine the X-direction forces that would be required on each of the bot's four wheels to achieve
        // the total frictional force and torque predicted by the kinematic model. Note that the magnitude of
        // total force on each wheel is sqrt(2) times abs(x-direction force). (ROBOT coordinate system)

        VectorF wheel_X_Forces = M_ForceRobotToWheel.multiplied(frictionForces);

        //If any of the wheel forces exceeds the product of muStatic*mass*gravity, reduce the magnitude
        //of that force to muKinetic*mass*gravity, keeping the direction the same

        for (int i=0; i<4; i++){
            float f = wheel_X_Forces.get(i);
            if (Math.abs(f) > maxWheelXForce) {
                wheel_X_Forces.put(i, maxWheelXForce * Math.signum(f));
            }
        }

        //Based on the adjusted forces at each wheel, determine net frictional force and torque on the bot,
        //Force is in ROBOT COORDINATE system

        frictionForces = M_ForceWheelToRobot.multiplied(wheel_X_Forces);

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
         * Compute linear and angular acceleration for use by SparkFunOTOS (World Coordinates)
         */
        Vector2 accelMetersPerSecSqr = force.quotient(chassisBody.getMass().getMass());
        double angularAccelRadiansPerSecSqr = torque / chassisBody.getMass().getInertia();

        /*
         * Update the sensors
         */
        imu.updateHeadingRadians(headingRadians);
        imuNew.updateHeadingRadians(headingRadians);
        odo.update(
                new Pose2D(DistanceUnit.METER, xMeters, yMeters, AngleUnit.RADIANS, headingRadians),
                new Pose2D(DistanceUnit.METER, velocityMetersPerSec.x, velocityMetersPerSec.y, AngleUnit.RADIANS, angularVelocityRadiansPerSec),
                new Pose2D(DistanceUnit.METER, accelMetersPerSecSqr.x, accelMetersPerSecSqr.y, AngleUnit.RADIANS, angularAccelRadiansPerSecSqr)
        );
        sparkFunOTOSInternal.update();

        // It isn't necessary to update the Pinpoint here; it is the responsibility of the user to update it.

        colorSensor.updateColor(x, y);

        final double piOver2 = Math.PI / 2.0;

        for (int i = 0; i < 4; i++) {
            double sensorHeading = AngleUtils.normalizeRadians(headingRadians + i * piOver2);
            distanceSensors[i].updateDistance(x - halfBotWidth * Math.sin(sensorHeading),
                    y + halfBotWidth * Math.cos(sensorHeading), sensorHeading);
        }

    }

    /**
     * Display the robot in the current orientation and position.
     */
    public synchronized void updateDisplay() {
        super.updateDisplay();
    }

    public void powerDownAndReset() {
        for (int i = 0; i < 4; i++) motors[i].stopAndReset();
        imu.close();
        odo.update(
                new Pose2D(DistanceUnit.METER, x/VirtualField.PIXELS_PER_METER, y/VirtualField.PIXELS_PER_METER, AngleUnit.RADIANS, headingRadians),
                new Pose2D(DistanceUnit.METER, 0, 0, AngleUnit.RADIANS, 0), new Pose2D(DistanceUnit.METER, 0, 0, AngleUnit.RADIANS, 0)
        );
        sparkFunOTOSInternal.update();
        goBildaPinpointDriverInternal.update();
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
        double botWidthMeters = botWidth / VirtualField.PIXELS_PER_METER;
        chassisFixture = chassisBody.addFixture(
                new org.dyn4j.geometry.Rectangle(botWidthMeters, botWidthMeters), 71.76, 0, 0);
        chassisRectangle = (org.dyn4j.geometry.Rectangle)chassisFixture.getShape();
        chassisFixture.setFilter(Filters.CHASSIS_FILTER);
        chassisBody.setMass(MassType.NORMAL);
        world.addBody(chassisBody);
    }

    @Override
    public synchronized void positionWithMouseClick(MouseEvent arg) {
        super.positionWithMouseClick(arg);
        odo.update(
                new Pose2D(DistanceUnit.METER, x/VirtualField.PIXELS_PER_METER, y/VirtualField.PIXELS_PER_METER, AngleUnit.RADIANS, headingRadians),
                new Pose2D(DistanceUnit.METER, 0, 0, AngleUnit.RADIANS, 0), new Pose2D(DistanceUnit.METER, 0, 0, AngleUnit.RADIANS, 0)
        );
        odo.setPosition(new Pose2D(DistanceUnit.METER, 0, 0, AngleUnit.RADIANS, 0));
        sparkFunOTOSInternal.update();
        goBildaPinpointDriverInternal.internalUpdate(false, false);
        goBildaPinpointDriverInternal.resetEncoders();
    }

}
