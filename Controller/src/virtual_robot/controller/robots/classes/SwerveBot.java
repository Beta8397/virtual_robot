package virtual_robot.controller.robots.classes;

import com.qualcomm.hardware.bosch.BNO055IMUImpl;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.hardware.configuration.MotorType;
import javafx.fxml.FXML;
import javafx.scene.shape.Rectangle;
import javafx.scene.transform.Rotate;
import virtual_robot.controller.BotConfig;
import virtual_robot.controller.VirtualBot;
import virtual_robot.controller.VirtualRobotController;
import virtual_robot.util.AngleUtils;
import virtual_robot.util.Vector2D;

/**
 * For internal use only. Represents a robot with four mechanum wheels, color sensor, four distance sensors,
 * a BNO055IMU, and a Servo-controlled arm on the back.
 *
 * MechanumBot is the controller class for the "mechanum_bot.fxml" markup file.
 *
 */
@BotConfig(name = "Swerve Bot", filename = "swerve_bot")
public class SwerveBot extends VirtualBot {

    MotorType motorType;
    private DcMotorExImpl[] motors = null;
    private CRServoImpl[] crServos = null;
    private DeadWheelEncoder[] steerEncoders = null;
    private BNO055IMUImpl imu = null;
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

    private double wheelCircumference;
    private double interWheelWidth;
    private double interWheelLength;

    //Vectors representing positions of each wheel relative to robot center, in field units
    private final Vector2D[] WHEEL_POS = new Vector2D[4];

    //Vector representing robot velocity in field units/sec
    private Vector2D velocity = new Vector2D(0, 0);
    //Scalar representing robot angular speed in radians/sec
    private double omega = 0;


    public SwerveBot(){
        super();
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
        //gyro = (VirtualRobotController.GyroSensorImpl)hardwareMap.gyroSensor.get("gyro_sensor");
        imu = hardwareMap.get(BNO055IMUImpl.class, "imu");
        colorSensor = (VirtualRobotController.ColorSensorImpl)hardwareMap.colorSensor.get("color_sensor");
        servo = (ServoImpl)hardwareMap.servo.get("back_servo");
        wheelCircumference = Math.PI * botWidth / 4.5;
        interWheelWidth = botWidth * 8.0 / 9.0;
        interWheelLength = botWidth * 7.0 / 9.0;

        WHEEL_POS[0] = new Vector2D(-interWheelWidth/2, -interWheelLength/2);
        WHEEL_POS[1] = new Vector2D(-interWheelWidth/2, interWheelLength/2);
        WHEEL_POS[2] = new Vector2D(interWheelWidth/2, interWheelLength/2);
        WHEEL_POS[3] = new Vector2D(interWheelWidth/2, -interWheelLength/2);

        hardwareMap.setActive(false);
        System.out.println("Exiting SwerveBot Constructor");
    }

    public void initialize(){
        //backServoArm = (Rectangle)displayGroup.getChildren().get(8);
        backServoArm.getTransforms().add(new Rotate(0, 37.5, 67.5));
    }

    protected void createHardwareMap(){
        motorType = MotorType.Neverest40;
        hardwareMap = new HardwareMap();
        String[] motorNames = new String[] {"back_left_motor", "front_left_motor", "front_right_motor", "back_right_motor"};
        String[] encoderNames = new String[] {"back_left_encoder", "front_left_encoder", "front_right_encoder", "back_right_encoder"};
        String[] crservoNames = new String[] {"back_left_crservo", "front_left_crservo", "front_right_crservo", "back_right_crservo"};
        for (String name: motorNames) hardwareMap.put(name, new DcMotorExImpl(motorType));
        for (String name: crservoNames) hardwareMap.put(name, new CRServoImpl(360));
        for (String name: encoderNames) hardwareMap.put(name, new DeadWheelEncoder(motorType));
        String[] distNames = new String[]{"front_distance", "left_distance", "back_distance", "right_distance"};
        for (String name: distNames) hardwareMap.put(name, controller.new DistanceSensorImpl());
        //hardwareMap.put("gyro_sensor", controller.new GyroSensorImpl());
        hardwareMap.put("imu", new BNO055IMUImpl(this, 10));
        hardwareMap.put("color_sensor", controller.new ColorSensorImpl());
        hardwareMap.put("back_servo", new ServoImpl());
    }

    public synchronized void updateStateAndSensors(double millis){

        /*
         *  This is not a realistic physics-based simulation. The virtual_robot simulator is kinematic. For the swerve bot,
         *  pure kinematic simulation isn't feasible, since the wheels can be aligned in such a way that lots of
         *  skidding is inevitable. This simulation ASSUMES that there is skidding, and assigns frictional forces to
         *  each wheel. Those forces depend on the speed at which the surface of that wheel is skidding relative
         *  to the floor (not true of real kinetic friction).
         */

        //Bot Mass is one botMass unit
        final double MAX_WHEEL_FORCE = 4 * botWidth;     //in botMass * fieldUnit/(sec^2)
        final double FORCE_COEFF = 4;         //in botMass/sec    i.e., (botMass*botWidth/sec^2)/(botWidth/sec)
        final double INERTIA = botWidth * botWidth / 6.0;       //in botMass * fieldUnit^2
        double t = millis / 1000.0;

        Vector2D totalForce = new Vector2D(0,0);
        double totalTorque = 0;

//        System.out.println("\n\nUpdateStateAndSensors Cycle");
//        System.out.println("  heading = " + Math.toDegrees(headingRadians) + "  velocity = " + velocity.x + "  " + velocity.y + "  omega = " + omega);

        for (int i = 0; i < 4; i++) {
            double deltaPos = motors[i].update(millis);
            steerEncoders[i].update(Math.toRadians(crServos[i].updatePositionDegrees(millis)), millis);
            double steer = Math.toRadians(crServos[i].getPositionDegrees());
            double s = deltaPos * wheelCircumference / motorType.TICKS_PER_ROTATION;
            if (i < 2) s = -s;
            Vector2D w = new Vector2D(-s * Math.sin(steer+headingRadians), s * Math.cos(steer+headingRadians));
            Vector2D v = velocity.added(WHEEL_POS[i].rotated(headingRadians+Math.PI/2).multiplied(omega));
            Vector2D skidVelocity = v.subtracted(w.divided(t));       //skid velocity in fieldUnit/sec
            Vector2D wheelForce = skidVelocity.multiplied(-FORCE_COEFF);          //frictional force on wheel in botMass*fieldUnit/sec^2
            double wheelForceMagnitude = wheelForce.length();
            if (wheelForceMagnitude > MAX_WHEEL_FORCE) wheelForce = wheelForce.multiplied(MAX_WHEEL_FORCE / wheelForceMagnitude);
            totalForce.add(wheelForce);
            Vector2D torqueArm = WHEEL_POS[i].rotated(headingRadians);
            double torque = torqueArm.cross(wheelForce);
            totalTorque += torque;                         //in botMass*fieldUnit^2/sec^2

//            System.out.printf("\n  wheel %d:  steer=%.1f  s=%.1f wx=%.1f wy=%.1f  vx=%.1f  vy=%.1f  skVX=%.1f  skVY=%.1f  Fx=%.1f  Fy=%.1f  torque = %.1f",
//                    i, Math.toDegrees(steer), s, w.x, w.y, v.x, v.y, skidVelocity.x, skidVelocity.y, wheelForce.x, wheelForce.y,
//                    torque);
        }

//        System.out.printf("\nTotal force = %.1f  %.1f   Total torque = %.1f", totalForce.x, totalForce.y, totalTorque);

        Vector2D dPos = totalForce.multiplied(0.5 * t * t).added(velocity.multiplied(t));      //in field units
        x += dPos.x;
        y += dPos.y;
        double dHeading = omega * t + 0.5 * totalTorque * t * t / INERTIA;  //in radians
        headingRadians += dHeading;
        if (headingRadians > Math.PI) {
            headingRadians -= 2.0 * Math.PI;
        } else if (headingRadians < -Math.PI) {
            headingRadians += 2.0 * Math.PI;
        }

        velocity = velocity.added(totalForce.multiplied(t));
        omega = omega + totalTorque * t / INERTIA;

        constrainToBoundaries();

        imu.updateHeadingRadians(headingRadians);

        colorSensor.updateColor(x, y);

        final double piOver2 = Math.PI / 2.0;

        for (int i = 0; i<4; i++){
            double sensorHeading = AngleUtils.normalizeRadians(headingRadians + i * piOver2);
            distanceSensors[i].updateDistance( x - halfBotWidth * Math.sin(sensorHeading),
                    y + halfBotWidth * Math.cos(sensorHeading), sensorHeading);
        }

    }

    @Override
    /**
     * Constrain robot to the boundaries X_MIN, X_MAX, Y_MIN, Y_MAX
     */
    protected void constrainToBoundaries(){

        double effectiveHalfBotWidth;    //Use this to keep corner of robot from leaving field
        if (headingRadians <= -Math.PI/2.0) effectiveHalfBotWidth = -halfBotWidth * (Math.sin(headingRadians) + Math.cos(headingRadians));
        else if (headingRadians <= 0) effectiveHalfBotWidth = halfBotWidth * (Math.cos(headingRadians) - Math.sin(headingRadians));
        else if (headingRadians <= Math.PI/2.0) effectiveHalfBotWidth = halfBotWidth * (Math.sin(headingRadians) + Math.cos(headingRadians));
        else effectiveHalfBotWidth = halfBotWidth * (Math.sin(headingRadians) - Math.cos(headingRadians));

        if (x >  (X_MAX - effectiveHalfBotWidth)) {
            x = X_MAX - effectiveHalfBotWidth;
            velocity.x = Math.min(velocity.x, 0);
        } else if (x < (X_MIN + effectiveHalfBotWidth)) {
            x = X_MIN + effectiveHalfBotWidth;
            velocity.x = Math.max(velocity.x, 0);
        }

        if (y > (Y_MAX - effectiveHalfBotWidth)){
            y = Y_MAX - effectiveHalfBotWidth;
            velocity.y = Math.min(velocity.y, 0);
        } else if (y < (Y_MIN + effectiveHalfBotWidth)) {
            y = Y_MIN + effectiveHalfBotWidth;
            velocity.y = Math.max(velocity.y, 0);
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
        velocity = new Vector2D(0, 0);
        omega = 0;
        imu.close();
    }


}
