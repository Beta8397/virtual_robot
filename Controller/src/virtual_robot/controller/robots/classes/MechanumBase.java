package virtual_robot.controller.robots.classes;

import com.qualcomm.hardware.bosch.BNO055IMUImpl;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorExImpl;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.MotorType;

import org.dyn4j.collision.CategoryFilter;
import org.dyn4j.dynamics.BodyFixture;
import org.dyn4j.dynamics.contact.Contact;
import org.dyn4j.dynamics.contact.SolvedContact;
import org.dyn4j.geometry.Vector2;
import org.dyn4j.world.BroadphaseCollisionData;
import org.dyn4j.world.ContactCollisionData;
import org.dyn4j.world.ManifoldCollisionData;
import org.dyn4j.world.NarrowphaseCollisionData;
import org.dyn4j.world.listener.CollisionListener;
import org.dyn4j.world.listener.ContactListener;
import org.firstinspires.ftc.robotcore.external.matrices.GeneralMatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.MatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import virtual_robot.controller.VRBody;
import virtual_robot.controller.VirtualBot;
import virtual_robot.controller.VirtualRobotController;
import virtual_robot.util.AngleUtils;

/**
 * For internal use only. Represents a robot with four mechanum wheels, color sensor, four distance sensors,
 * a BNO055IMU, and a Servo-controlled arm on the back.
 */
public class MechanumBase extends VirtualBot {

    public final MotorType MOTOR_TYPE;
    private DcMotorExImpl[] motors = null;
    //private VirtualRobotController.GyroSensorImpl gyro = null;
    private BNO055IMUImpl imu = null;
    private VirtualRobotController.ColorSensorImpl colorSensor = null;
    private VirtualRobotController.DistanceSensorImpl[] distanceSensors = null;

    private double wheelCircumference;
    protected double gearRatioWheel = 1.0;
    private double interWheelWidth;
    private double interWheelLength;
    private double wlAverage;

    private double[][] tWR; //Transform from wheel motion to robot motion (KINETIC MODEL)



    GeneralMatrixF M_ForceWheelToRobot; // Converts from individual wheel forces to total force/torque on robot
    MatrixF M_ForceRobotToWheel;  // Converts from total force/torque on robot to individual wheel forces

    protected final float FIELD_FRICTION_COEFF = 100.0f;  // For now, use ridiculously high friction coefficient
    protected float maxWheelXForce; // Need to assign this in the initialize method.

    /**
     * No-param constructor. Uses the default motor type of Neverest 40
     */
    public MechanumBase() {
        super();
        MOTOR_TYPE = MotorType.Neverest40;
    }

    public MechanumBase(MotorType driveMotorType){
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
        colorSensor = (VirtualRobotController.ColorSensorImpl) hardwareMap.colorSensor.get("color_sensor");
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

        float RRt2 = 0.5f * (float)Math.sqrt(interWheelLength*interWheelLength + interWheelWidth*interWheelWidth) * (float)Math.sqrt(2.0);

        M_ForceWheelToRobot = new GeneralMatrixF(4, 4, new float[]{
                1, 1, 1, 1,
                -1, 1, -1, 1,
                RRt2, -RRt2, -RRt2, RRt2,
                1, 1, -1, -1});

        M_ForceRobotToWheel = M_ForceWheelToRobot.inverted();

        // Maximum possible frictional force (in robot-X direction) between field and any individual robot wheel.
        // Note the division by 4 (assumes each wheel gets 1/4 of robot mass) and the division by sqrt(2) (because
        // the X-direction force is 1/sqrt(2) times the total friction force on the wheel.
        maxWheelXForce = (float)(9.8 * chassisBody.getMass().getMass() * FIELD_FRICTION_COEFF / (4.0 * Math.sqrt(2)));
        System.out.println("Max wheel X force = " + maxWheelXForce);

        world.addCollisionListener(new CollisionListener<VRBody, BodyFixture>() {
            @Override
            public boolean collision(BroadphaseCollisionData<VRBody, BodyFixture> collision) {
                System.out.println("BroadPhase Listener");
                return true;
            }

            @Override
            public boolean collision(NarrowphaseCollisionData<VRBody, BodyFixture> collision) {
                System.out.println("NarrowPhase Listener");
                return true;
            }

            @Override
            public boolean collision(ManifoldCollisionData<VRBody, BodyFixture> collision) {
                long cat1 = ((CategoryFilter)collision.getFixture1().getFilter()).getCategory();
                long cat2 = ((CategoryFilter)collision.getFixture2().getFilter()).getCategory();
                System.out.println("Manifold Listener:  cat1 = " + cat1 + "  cat2 = " + cat2);
                return true;
            }
        });

        world.addContactListener(new ContactListener<VRBody>() {
            @Override
            public void begin(ContactCollisionData<VRBody> collision, Contact contact) {
                System.out.println("Contact begin");
            }

            @Override
            public void persist(ContactCollisionData<VRBody> collision, Contact oldContact, Contact newContact) {
                System.out.println("Contact persist");
            }

            @Override
            public void end(ContactCollisionData<VRBody> collision, Contact contact) {
                System.out.println("Contace end");
            }

            @Override
            public void destroyed(ContactCollisionData<VRBody> collision, Contact contact) {
                System.out.println("Contact destroyed");
            }

            @Override
            public void collision(ContactCollisionData<VRBody> collision) {
                System.out.println("Contact collision");
            }

            @Override
            public void preSolve(ContactCollisionData<VRBody> collision, Contact contact) {
                System.out.println("Contact preSolve");
            }

            @Override
            public void postSolve(ContactCollisionData<VRBody> collision, SolvedContact contact) {
                System.out.println("Contact postSolve");
            }
        });

        hardwareMap.setActive(false);

    }

    protected void createHardwareMap() {
        hardwareMap = new HardwareMap();
        String[] motorNames = new String[]{"back_left_motor", "front_left_motor", "front_right_motor", "back_right_motor"};
        for (String name : motorNames) hardwareMap.put(name, new DcMotorExImpl(MOTOR_TYPE));
        String[] distNames = new String[]{"front_distance", "left_distance", "back_distance", "right_distance"};
        for (String name : distNames) hardwareMap.put(name, controller.new DistanceSensorImpl());
        //hardwareMap.put("gyro_sensor", controller.new GyroSensorImpl());
        hardwareMap.put("imu", new BNO055IMUImpl(this, 10));
        hardwareMap.put("color_sensor", controller.new ColorSensorImpl());
    }

    public synchronized void updateStateAndSensors(double millis) {

        //Update current position (pixel units) by obtaining it from physics Body and converting to pixels
        x = chassisBody.getTransform().getTranslationX() * FIELD.PIXELS_PER_METER;
        y = chassisBody.getTransform().getTranslationY() * FIELD.PIXELS_PER_METER;
        headingRadians = chassisBody.getTransform().getRotationAngle();

//        System.out.println("Updated Position (Pixel Units): x = " + x + "  y = " + y + " heading = " + headingRadians);

        /*
         * Calculate the force (from friction with the floor) to be applied to the robot chassis during the next
         * physics update.
         *
         * The first step is to compute the tentative change in robot position using the KINEMATIC model.
         */
        double[] deltaPos = new double[4];
        double[] w = new double[4];

        for (int i = 0; i < 4; i++) {
            deltaPos[i] = motors[i].update(millis);
            w[i] = deltaPos[i] * wheelCircumference * gearRatioWheel / motors[i].MOTOR_TYPE.TICKS_PER_ROTATION;
            if (i < 2) w[i] = -w[i];
        }

        double[] robotDeltaPos = new double[]{0, 0, 0, 0};
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                robotDeltaPos[i] += tWR[i][j] * w[j];
            }
        }

        // TENTATIVE change in position, in ROBOT COORDINATES (PIXEL UNITS)
        double dxR = robotDeltaPos[0];
        double dyR = robotDeltaPos[1];
        double headingChange = robotDeltaPos[2];
        double avgHeading = headingRadians + headingChange / 2.0;
//        System.out.println("dxR = " + dxR + "  dyR = " + dyR + "  headingChange = " + headingChange);

        double sin = Math.sin(avgHeading);
        double cos = Math.cos(avgHeading);

        // TENTATIVE change in position, in WORLD COORDINATES (METERS)
        double dX = (dxR * cos - dyR * sin) / FIELD.PIXELS_PER_METER;
        double dY = (dxR * sin + dyR * cos) / FIELD.PIXELS_PER_METER;

        /* Determine the force and torque (WORLD COORDS) that would be required to achieve the changes predicted by
         * the kinematic model.
         *
         * Note: dX, dY, and dHeading are based on average speeds during the interval, so we can calculate force
         * and torque using:
         *
         *       d(Position) = v0*t + 0.5*(F/m)*t*t
         *       d(Heading) = omega0*t + 0.5*(Torque/I)*t*t
         *
         *       or,
         *
         *       F = 2m( d(Position) - v0*t ) / (t*t)
         *       Torque = 2I( d(Heading) - omega0*t ) / (t*t)
         */

        double t = millis / 1000.0;
        double tSqr = t * t;
        Vector2 dPos = new Vector2(dX, dY);
        Vector2 v0 = chassisBody.getLinearVelocity();
        Vector2 force = dPos.difference(v0.product(t)).product(2.0*chassisBody.getMass().getMass()/tSqr);
        double angVel = chassisBody.getAngularVelocity();
        double torque = 2.0 * chassisBody.getMass().getInertia() * (headingChange - angVel*t) / tSqr;

        //Convert the tentative total force to the ROBOT COORDINATE system

        double sinHd = Math.sin(headingRadians);
        double cosHd = Math.cos(headingRadians);

        float fXR = (float)(force.x*cosHd + force.y*sinHd);
        float fYR = (float)(-force.x*sinHd + force.y*cosHd);

        // VectorF containing total force and torque required on bot to achieve the tentative position change,
        // in robot coordinate system
        VectorF totalForce = new VectorF(fXR, fYR, (float)torque, 0);

//        System.out.println("Tentative Force (Robot):  fxR = " + fXR + "  fYR = " + fYR + "  torque = " + torque);

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

//        System.out.println("Adjusted total force (Robot)  fXR = " + frictionForces.get(0) + "  fYR = "
//         + frictionForces.get(1) + "  torque = " + frictionForces.get(2));

        // Convert these adjusted friction forces to WORLD COORDINATES  and put into the original
        // force and torque variables
        force = new Vector2(frictionForces.get(0)*cosHd - frictionForces.get(1)*sinHd,
                frictionForces.get(0)*sinHd + frictionForces.get(1)*cosHd);
        torque = frictionForces.get(2);

//        System.out.println("Final force (WORLD):  FX = " + force.x + "  FY = " + force.y + "  torque = " + torque);

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

        colorSensor.updateColor(x, y);

        final double piOver2 = Math.PI / 2.0;

        for (int i = 0; i < 4; i++) {
            double sensorHeading = AngleUtils.normalizeRadians(headingRadians + i * piOver2);
            distanceSensors[i].updateDistance(x - halfBotWidth * Math.sin(sensorHeading),
                    y + halfBotWidth * Math.cos(sensorHeading), sensorHeading);
        }

    }

    public synchronized void updateDisplay() {
        super.updateDisplay();
    }

    public void powerDownAndReset() {
        for (int i = 0; i < 4; i++) motors[i].stopAndReset();
        imu.close();
    }


}
