package virtual_robot.robots.classes;

import com.qualcomm.hardware.CommonOdometry;
import com.qualcomm.hardware.bosch.BNO055IMUImpl;
import com.qualcomm.hardware.bosch.BNO055IMUNew;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriverInternal;
import com.qualcomm.robotcore.hardware.*;
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
import virtual_robot.controller.Filters;
import virtual_robot.controller.VirtualBot;
import virtual_robot.controller.VirtualField;
import virtual_robot.controller.VirtualRobotController;
import virtual_robot.util.AngleUtils;

/**
 * For internal use only. Base class for a physics-based robot with four mecanum wheels, color sensor,
 * four distance sensors, and a BNO055IMU.
 */
public abstract class DynamicMecanumBase extends VirtualBot {

    public final MotorType MOTOR_TYPE;
    private DcMotorExDynImpl[] motors = null;
    private BNO055IMUImpl imu = null;
    BNO055IMUNew imuNew = null;
    private CommonOdometry odo = CommonOdometry.getInstance();
    private GoBildaPinpointDriverInternal goBildaPinpointDriverInternal = null;
    private VirtualRobotController.ColorSensorImpl colorSensor = null;
    private VirtualRobotController.DistanceSensorImpl[] distanceSensors = null;

    /*
     * Gear ratio for any external gears added in drive train. For now, this is just 1.0. Could easily
     * add a constructor to MecanumPhysicsBase that allows this to be set to some other value.
     */
    protected double gearRatioWheel = 1.0;

    /*
     * Robot geometry
     */

    private final double wheelRadiusMeters = 2.0 / VirtualField.INCHES_PER_METER;
    private final double wheelInertia = 0.236 * (9.0/16.0) * wheelRadiusMeters * wheelRadiusMeters;
    private final double wheelCircumferenceMeters = Math.PI * 2.0 * wheelRadiusMeters;
    private final double wlAverageMeters = 15.0 / VirtualField.INCHES_PER_METER;

    private MatrixF motorTorqueToRobotForces;

    /**
     * No-param constructor. Uses the default motor type of Neverest 40
     */
    public DynamicMecanumBase() {
        super();
        MOTOR_TYPE = MotorType.Neverest40;
    }

    public DynamicMecanumBase(MotorType driveMotorType){
        super();
        MOTOR_TYPE = driveMotorType;
    }

    public void initialize(){
        super.initialize();
        hardwareMap.setActive(true);
        motors = new DcMotorExDynImpl[]{
                (DcMotorExDynImpl) hardwareMap.get(DcMotorEx.class, "back_left_motor"),
                (DcMotorExDynImpl) hardwareMap.get(DcMotorEx.class, "front_left_motor"),
                (DcMotorExDynImpl) hardwareMap.get(DcMotorEx.class, "front_right_motor"),
                (DcMotorExDynImpl) hardwareMap.get(DcMotorEx.class, "back_right_motor")
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

        goBildaPinpointDriverInternal = hardwareMap.get(GoBildaPinpointDriverInternal.class, "pinpoint");

        hardwareMap.setActive(false);

        /*
         * Compute the motorTorqueToRobotForces matrix. This converts from a vector containing
         * the torque output of the four drive motors to the net forces and torque on the robot.
         */

        // Kinetics matrix K: transform from bot speeds to wheel speeds (in clockwise dir)
        GeneralMatrixF K = new GeneralMatrixF(4, 4, new float[]{
                1, -1, (float)wlAverageMeters, 1,
                -1, -1, (float)wlAverageMeters, -1,
                -1, 1, (float)wlAverageMeters, 1,
                1, 1, (float)wlAverageMeters, -1
        });

        // Mass matrix M: diagonal matrix containing robot mass and moment of inertia
        MatrixF M = GeneralMatrixF.diagonalMatrix(new VectorF(
                (float)chassisBody.getMass().getMass(), (float)chassisBody.getMass().getMass(),
                (float)chassisBody.getMass().getInertia(), (float)chassisBody.getMass().getMass()
        ));

        MatrixF temp1 = K.multiplied(M.inverted().multiplied(K.transposed()))
                .multiplied((float)(wheelInertia / wheelRadiusMeters));
        MatrixF temp2 = GeneralMatrixF.identityMatrix(4)
                .multiplied((float)wheelRadiusMeters);

        motorTorqueToRobotForces = K.transposed().multiplied((temp1.added(temp2)).inverted());

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
            hardwareMap.put(motorNames[i], new DcMotorExDynImpl(MOTOR_TYPE, motorController0, i));
        }
        String[] distNames = new String[]{"front_distance", "left_distance", "back_distance", "right_distance"};
        for (String name : distNames) hardwareMap.put(name, controller.new DistanceSensorImpl());
        hardwareMap.put("imu", new BNO055IMUImpl(this, 10));
        hardwareMap.put("imu", new BNO055IMUNew(this, 10));
        hardwareMap.put("color_sensor", controller.new ColorSensorImpl());
        hardwareMap.put("pinpoint", new GoBildaPinpointDriverInternal());
    }

    /**
     * Update the position of the robot on the field, as well as the distance, BNO055IMU, and color sensors.
     *
     * Updating robot position involves:
     *     1) Get new position and orientation from the dyn4j physics engine.
     *     2) Using dynamic model, based on motor raw powers and current robot velocity,
     *        "preload" the chassis body with force and torque to be applied
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
         * Get updated velocities in WORLD coordinates
         */
        Vector2 velocityMetersPerSec = chassisBody.getLinearVelocity();
        double vxMetersPerSec = velocityMetersPerSec.x;
        double vyMetersPerSec = velocityMetersPerSec.y;
        double angularVelocityRadiansPerSec = chassisBody.getAngularVelocity();

        double sinHd = Math.sin(headingRadians);
        double cosHd = Math.cos(headingRadians);

        /*
         * Compute new robot velocities in ROBOT COORDINATE SYSTEM in meters/sec
         */
        double vxRobot = vxMetersPerSec*cosHd + vyMetersPerSec*sinHd;
        double vyRobot = -vxMetersPerSec*sinHd + vyMetersPerSec*cosHd;

        /*
         * Compute the actual wheel speeds in -1 to +1 range, with positive values indicating
         * clockwise rotation.
         */

        double[] wSpd = new double[4];
        wSpd[0] = vxRobot - vyRobot + angularVelocityRadiansPerSec*wlAverageMeters;
        wSpd[1] = -vxRobot - vyRobot + angularVelocityRadiansPerSec*wlAverageMeters;
        wSpd[2] = -vxRobot + vyRobot + angularVelocityRadiansPerSec*wlAverageMeters;
        wSpd[3] = vxRobot + vyRobot + angularVelocityRadiansPerSec*wlAverageMeters;
        for (int i=0; i<4; i++){
            wSpd[i] *= 60.0 / (wheelCircumferenceMeters * MOTOR_TYPE.MAX_RPM);
        }

        /*
         * Update motor speeds and obtain motor torque outputs (in Newton-meters)
         */
        float[] tauArray = new float[4];
        for (int i=0; i<4; i++){
            tauArray[i] = (float)motors[i].update(millis, wSpd[i]);
        }
        VectorF tauVec = new VectorF(tauArray);

        /*
         * Compute forces (from drive motors) on robot in ROBOT COORDINATE SYSTEM
         */
//        double fxR = (tauArray[0] - tauArray[1] - tauArray[2] + tauArray[3]) / wheelRadiusMeters;
//        double fyR = (-tauArray[0] - tauArray[1] + tauArray[2] + tauArray[3]) / wheelRadiusMeters;
//        double torque = (tauArray[0] + tauArray[1] + tauArray[2] + tauArray[3]) * wlAverageMeters / wheelRadiusMeters;

        VectorF forcesRobot = motorTorqueToRobotForces.multiplied(tauVec);

        /*
         * Compute forces (from drive motors) on robot in FIELD COORDINATE SYSTEM
         */
//        double fx = fxR * sinHd + fyR * cosHd;
//        double fy = -fxR * cosHd + fyR * sinHd;
        double fx = forcesRobot.get(0) * cosHd - forcesRobot.get(1) * sinHd;
        double fy = forcesRobot.get(0) * sinHd + forcesRobot.get(1) * cosHd;
        double torque = forcesRobot.get(2);

        Vector2 force = new Vector2(fx, fy);

        /*
         * Apply the force and torque to the chassisBody
         *
         * Note:  We are only applying the forces from the floor, NOT the collision forces. The
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
                new org.dyn4j.geometry.Rectangle(botWidthMeters, botWidthMeters), 47.84, 0, 0);
        chassisRectangle = (org.dyn4j.geometry.Rectangle)chassisFixture.getShape();
        chassisFixture.setFilter(Filters.CHASSIS_FILTER);
        chassisBody.setMass(MassType.NORMAL);
        chassisBody.setLinearDamping(1);
        chassisBody.setAngularDamping(1);
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
        goBildaPinpointDriverInternal.internalUpdate(false, false);
        goBildaPinpointDriverInternal.resetEncoders();
    }

}
