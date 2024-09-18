package virtual_robot.robots.classes;

import com.qualcomm.hardware.bosch.BNO055IMUImpl;
import com.qualcomm.hardware.bosch.BNO055IMUNew;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.hardware.configuration.MotorType;
import javafx.scene.transform.Rotate;
import org.dyn4j.dynamics.Body;
import org.dyn4j.geometry.MassType;
import org.dyn4j.geometry.Vector2;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import virtual_robot.config.Config;
import virtual_robot.controller.Filters;
import virtual_robot.controller.VirtualBot;
import virtual_robot.controller.VirtualField;
import virtual_robot.controller.VirtualRobotController;
import virtual_robot.util.AngleUtils;

/**
 * For internal use only. Abstract base class for a physics-based two wheel robot.
 */
public class TwoWheelPhysicsBase extends VirtualBot {

    private final MotorType MOTOR_TYPE = MotorType.Neverest40;
    private DcMotorExImpl leftMotor = null;
    private DcMotorExImpl rightMotor = null;
    private BNO055IMUImpl imu = null;
    private BNO055IMUNew imuNew = null;
    private VirtualRobotController.ColorSensorImpl colorSensor = null;
    private VirtualRobotController.DistanceSensorImpl[] distanceSensors = null;

    private double wheelCircumference;
    private double interWheelDistance;

    private double maxWheelForce;

    public TwoWheelPhysicsBase(){
        super();
    }

    public void initialize(){
        super.initialize();

        hardwareMap.setActive(true);
        leftMotor = (DcMotorExImpl)hardwareMap.get(DcMotorEx.class, "left_motor");
        rightMotor = (DcMotorExImpl)hardwareMap.get(DcMotorEx.class, "right_motor");
        distanceSensors = new VirtualRobotController.DistanceSensorImpl[]{
                hardwareMap.get(VirtualRobotController.DistanceSensorImpl.class, "front_distance"),
                hardwareMap.get(VirtualRobotController.DistanceSensorImpl.class, "left_distance"),
                hardwareMap.get(VirtualRobotController.DistanceSensorImpl.class, "back_distance"),
                hardwareMap.get(VirtualRobotController.DistanceSensorImpl.class, "right_distance")
        };
        imu = hardwareMap.get(BNO055IMUImpl.class, "imu");
        imuNew = hardwareMap.get(BNO055IMUNew.class, "imu");
        colorSensor = (VirtualRobotController.ColorSensorImpl)hardwareMap.colorSensor.get("color_sensor");
        wheelCircumference = Math.PI * botWidth / 4.5;
        interWheelDistance = botWidth * 8.0 / 9.0;
        hardwareMap.setActive(false);

        // Maximum possible frictional force between field and any individual robot wheel.
        // Note the division by 2 (assumes each wheel gets 1/2 of robot mass).
        maxWheelForce = 9.8 * chassisBody.getMass().getMass() * Config.FIELD_FRICTION_COEFF / 2.0;
    }

    protected void createHardwareMap(){
        hardwareMap = new HardwareMap();
        hardwareMap.put("left_motor", new DcMotorExImpl(MOTOR_TYPE, motorController0, 0));
        hardwareMap.put("right_motor", new DcMotorExImpl(MOTOR_TYPE, motorController0, 1));
        String[] distNames = new String[]{"front_distance", "left_distance", "back_distance", "right_distance"};
        for (String name: distNames) hardwareMap.put(name, controller.new DistanceSensorImpl());
        hardwareMap.put("imu", new BNO055IMUImpl(this, 10));
        hardwareMap.put("imu", new BNO055IMUNew(this, 10));
        hardwareMap.put("color_sensor", controller.new ColorSensorImpl());
    }

    public synchronized void updateStateAndSensors(double millis){

        /*
         * Get updated position and heading from the dyn4j body (chassisBody)
         */
        x = chassisBody.getTransform().getTranslationX() * VirtualField.PIXELS_PER_METER;
        y = chassisBody.getTransform().getTranslationY() * VirtualField.PIXELS_PER_METER;
        headingRadians = chassisBody.getTransform().getRotationAngle();

        /*
         * Based on current motor speeds, determine the target robot linear and angular
         * speed to be achieved at the next physics update
         */
        leftMotor.update(millis);
        rightMotor.update(millis);
        double leftWheelSpeed = leftMotor.getVelocity(AngleUnit.RADIANS) * wheelCircumference / (2.0 * Math.PI);
        double rightWheelSpeed = rightMotor.getVelocity(AngleUnit.RADIANS) * wheelCircumference / (2.0 * Math.PI);
        boolean ltRev = leftMotor.getDirection() == DcMotorSimple.Direction.REVERSE;
        boolean rtRev = rightMotor.getDirection() == DcMotorSimple.Direction.REVERSE;
        if (MOTOR_TYPE.REVERSED && ltRev || !MOTOR_TYPE.REVERSED && !ltRev) leftWheelSpeed = -leftWheelSpeed;
        if (MOTOR_TYPE.REVERSED && !rtRev || !MOTOR_TYPE.REVERSED && rtRev) rightWheelSpeed = -rightWheelSpeed;

        double targetRobotSpeed = 0.5 * (leftWheelSpeed + rightWheelSpeed) / VirtualField.PIXELS_PER_METER;
        double targetAngularSpeed = (rightWheelSpeed - leftWheelSpeed) / interWheelDistance;

        /*
         * Compute an estimated final heading. This is used only to convert the target final robot velocity
         * from robot coordinate system to world coordinate system.
         */
        double t = millis / 1000.0;
        double estFinalHeading = headingRadians + 0.5 * (chassisBody.getAngularVelocity() + targetAngularSpeed) * t;
        double sinFinal = Math.sin(estFinalHeading);
        double cosFinal = Math.cos(estFinalHeading);

        /*
         * Convert target final robot velocity from robot coordinate system to world coordinate system.
         */
        Vector2 estFinalVelocity = new Vector2(-targetRobotSpeed*sinFinal, targetRobotSpeed*cosFinal);

        /*
         * Compute the force and torque that would be required to achieve the target changes in robot velocity and
         * angular speed (F = ma,  tau = I*alpha)
         */
        Vector2 force = estFinalVelocity.difference(chassisBody.getLinearVelocity()).product(chassisBody.getMass().getMass()/t);
        double torque = (targetAngularSpeed - chassisBody.getAngularVelocity()) * chassisBody.getMass().getInertia()/t;

        /*
         * Convert the proposed force from world to robot coordinate system.
         */
        double sinHd = Math.sin(headingRadians);
        double cosHd = Math.cos(headingRadians);

        float fXR = (float)(force.x*cosHd + force.y*sinHd);
        float fYR = (float)(-force.x*sinHd + force.y*cosHd);

        /*
         * Compute the frictional force that would be required on each wheel, in robot coordinate system, to
         * achieve the total force and torque requested. NOTE: will assume equal contribution to the linear
         * force from each wheel, even though this will generally not be true.
         */
        double interWheelDistanceMeters = interWheelDistance / VirtualField.PIXELS_PER_METER;
        Vector2 fRight = new Vector2(fXR/2.0, fYR/2.0 + torque/interWheelDistanceMeters);
        Vector2 fLeft = new Vector2(fXR/2.0, fYR/2.0 - torque/interWheelDistanceMeters);

        /*
         * If the frictional force proposed on either the right or left wheel is greater than the maximum
         * allowed frictional force per wheel, scale the force down so that its magnitude equals the maximum
         * frictional force.
         */
        fRight.setMagnitude(Math.min(fRight.getMagnitude(), maxWheelForce));
        fLeft.setMagnitude(Math.min(fLeft.getMagnitude(), maxWheelForce));

        /*
         * Recompute the torque on bot based on the adjusted wheel forces in robot coordinate system
         */
        torque = 0.5 * interWheelDistanceMeters * (fRight.y - fLeft.y);

        /*
         * Convert adjusted wheel forces from robot to world coordinate system, then compute total force
         * to apply to bot.
         */
        fRight.rotate(headingRadians);
        fLeft.rotate(headingRadians);
        Vector2 fTotal = fRight.sum(fLeft);

        /*
         * Apply the torque and total force to the chassis body, to take effect during the next
         * physics world update.
         */
        chassisBody.applyForce(fTotal);
        chassisBody.applyTorque(torque);

        /*
         * Update sensors.
         */
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
    }

    public void powerDownAndReset(){
        leftMotor.stopAndReset();
        rightMotor.stopAndReset();
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
        double botWidthMeters = botWidth / VirtualField.PIXELS_PER_METER;
        chassisFixture = chassisBody.addFixture(
                new org.dyn4j.geometry.Rectangle(botWidthMeters, botWidthMeters), 71.76, 0, 0);
        chassisRectangle = (org.dyn4j.geometry.Rectangle)chassisFixture.getShape();
        chassisFixture.setFilter(Filters.CHASSIS_FILTER);
        chassisBody.setMass(MassType.NORMAL);
        world.addBody(chassisBody);
        System.out.println("Chassis Mass = " + chassisBody.getMass().getMass());
        System.out.println("Chassis Inertia = " + chassisBody.getMass().getInertia());
    }


}
