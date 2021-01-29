package virtual_robot.controller.robots.classes;

import com.qualcomm.hardware.bosch.BNO055IMUImpl;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.CRServoImpl;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorExImpl;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DigitalChannelImpl;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImpl;
import com.qualcomm.robotcore.hardware.configuration.MotorType;

import java.awt.SystemTray;
import java.util.ArrayList;
import java.util.List;

import javafx.fxml.FXML;
import javafx.scene.shape.Rectangle;
import javafx.scene.transform.Rotate;
import javafx.scene.transform.Scale;
import javafx.scene.transform.Translate;
import virtual_robot.controller.BotConfig;
import virtual_robot.controller.VirtualBot;
import virtual_robot.controller.VirtualField;
import virtual_robot.controller.VirtualGameElement;
import virtual_robot.controller.VirtualRobotController;
import virtual_robot.controller.game_elements.classes.Ring;
import virtual_robot.controller.game_elements.classes.WobbleGoal;
import virtual_robot.controller.robots.GameElementControlling;
import virtual_robot.util.AngleUtils;
import virtual_robot.util.Vector2D;

/**
 * For internal use only. Represents a robot with four omni wheels, shooter motor, shooter servo,
 * intake motor, arm motor, hand servo, distance sensors, and a BNO055IMU.
 * <p>
 * BoPBot is modeled after the Team 11528 Bots of Prey 2020-2021 Ultimate Goal competition robot.
 * <p>
 * BoPBot is the controller class for the "bop_bot.fxml" markup file.
 */
@BotConfig(name = "BoP Bot", filename = "bop_bot")
public class BoPBot extends VirtualBot implements GameElementControlling {

    public static final double HOPPER_R_INCHES = 6.0;
    public static final double HOPPER_ANGLE_DEG = 270.0;

    public static final double SHOOTER_MAX_INCHES_PER_SEC = 315.0; // ~8 m/s
    public static final double SHOOTER_OFFSET_DEG = 100;

    public static final double SHOOTER_EXIT_R_INCHES = 6.0;
    public static final double SHOOTER_EXIT_ANGLE = 0.0;

    public static final double INTAKE_R_INCHES = 9.0;
    public static final double INTAKE_ANGLE_DEG = 90.0;
    public static final double INTAKE_DIR_TOLERANCE_DEG = 5.0;      // determines how accurate the drive direction must be to pick up a ring
    public static final double INTAKE_ANGLE_TOLERANCE_DEG = 10.0;   // determines how accurate the alignment to the ring must be
    public static final double INTAKE_DIST_TOLERANCE_INCHES = 13.5; // the distance to the ring at pick-up
    public static final long INTAKE_TO_HOPPER_TIME_MILLIS = 1000L;  // milliseconds that a ring blocks the intake from collecting another ring

    public static final double WOBBLE_GRAB_TURN_FRACTION = 0.34; // fraction of turn at which wobble goal can be grabbed
    
    private final MotorType MOTOR_TYPE = MotorType.Neverest20;
    private DcMotorExImpl[] wheelMotors = null;
    private final MotorType SHOOTER_MOTOR_TYPE = MotorType.RevUltraPlanetaryOneToOne;
    private DcMotorExImpl shooterMotor = null;
    private final MotorType INTAKE_MOTOR_TYPE = MotorType.Gobilda137; // actual motor is goBUILDA 5202 planetary gear motor
    private DcMotorExImpl intakeMotor = null;
    private ServoImpl shooterServo = null;
    private final MotorType ARM_MOTOR_TYPE = MotorType.Neverest60;
    private DcMotorExImpl armMotor = null;
    private ServoImpl handServo = null;
    private DigitalChannel armSwitch = null;
    private BNO055IMUImpl imu = null;
    private VirtualRobotController.DistanceSensorImpl[] distanceSensors = null;
    private VirtualRobotController.DistanceSensorImpl intakeSensor = null;
    private VirtualRobotController.DistanceSensorImpl ringDetector = null;

    /*
    Variables representing graphical UI nodes that we will need to manipulate. The @FXML annotation will
    cause these variables to be instantiated automatically during loading of the bop_bot.fxml file. The
    fxml file must declare fx:id attributes for the Rectangles that represent the arm, hand, and both fingers.
    For example, the attribute for the arm would be:  fx:id="arm"
     */
    @FXML private Rectangle shooterFlyWheel;
    @FXML private Rectangle shooterIndexer;
    @FXML private Rectangle intake1;
    @FXML private Rectangle intake2;
    @FXML private Rectangle arm;            // The arm. Must be able to extend/retract (i.e., scale) in Y-dimension.
    @FXML private Rectangle hand;           // The hand. Must move in Y-dimension as arm extends/retracts.
    @FXML private Rectangle leftFinger;     // Fingers must open and close based on position of hand servo.
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
    private volatile double armScale = 1.0;

    private double wheelCircumference;

    private double[][] tWR; //Transform from wheel motion to robot motion

    private Double directionOfTravel = null; // angle in radians or null if bot not moving

    private WobbleGoal wobble_goal = null; // wobble goal currently controlled by the robot
    private final List<Ring> ringsInHopper = new ArrayList<>(); // rings currently controlled by the robot
    private boolean readyToShoot = false;
    private Ring ringInIntake = null;
    private int directionOfRing = 1;
    private boolean isRingInIntake = false;
    private int ringWidth = 0;
    private boolean ringPastSensor = false;
    private long intakeTimeMillis = 0L; // timestamp of when ring entered intake
    private long lastTimeMillis = 0L;
    private double positionInIntake = 0;

    public BoPBot() {
        super();
    }

    public void initialize() {
        super.initialize();

        hardwareMap.setActive(true);
        wheelMotors = new DcMotorExImpl[]{
                (DcMotorExImpl) hardwareMap.get(DcMotorEx.class, "blMotor"),
                (DcMotorExImpl) hardwareMap.get(DcMotorEx.class, "flMotor"),
                (DcMotorExImpl) hardwareMap.get(DcMotorEx.class, "frMotor"),
                (DcMotorExImpl) hardwareMap.get(DcMotorEx.class, "brMotor")
        };

        shooterMotor = (DcMotorExImpl) hardwareMap.get(DcMotorEx.class, "wheelMotor");
        shooterServo = (ServoImpl) hardwareMap.servo.get("indexServo");
        intakeMotor = (DcMotorExImpl) hardwareMap.get(DcMotorEx.class, "intakeMotor");
        intakeSensor = hardwareMap.get(VirtualRobotController.DistanceSensorImpl.class, "intakeSensor");
        armMotor = (DcMotorExImpl) hardwareMap.get(DcMotorEx.class, "wobbleGrabberArm");
        handServo = (ServoImpl) hardwareMap.servo.get("wobbleGrabberClaw");
        armSwitch = (DigitalChannelImpl) hardwareMap.get(DigitalChannel.class, "armSwitch");
        ringDetector = hardwareMap.get(VirtualRobotController.DistanceSensorImpl.class, "ringDetector");

        distanceSensors = new VirtualRobotController.DistanceSensorImpl[]{
                hardwareMap.get(VirtualRobotController.DistanceSensorImpl.class, "front_distance"),
                hardwareMap.get(VirtualRobotController.DistanceSensorImpl.class, "left_distance"),
                hardwareMap.get(VirtualRobotController.DistanceSensorImpl.class, "back_distance"),
                hardwareMap.get(VirtualRobotController.DistanceSensorImpl.class, "right_distance")
        };

        imu = hardwareMap.get(BNO055IMUImpl.class, "imu");
        wheelCircumference = Math.PI * botWidth / 4.5;
        double sqrt2 = Math.sqrt(2);
        double wheelBaseRadius = botWidth * (1.0 / sqrt2 - 5.0 / 36.0);

        tWR = new double[][]{
                {-0.25 * sqrt2, 0.25 * sqrt2, -0.25 * sqrt2, 0.25 * sqrt2},
                {0.25 * sqrt2, 0.25 * sqrt2, 0.25 * sqrt2, 0.25 * sqrt2},
                {-0.25 / wheelBaseRadius, -0.25 / wheelBaseRadius, 0.25 / wheelBaseRadius, 0.25 / wheelBaseRadius},
                {-0.25, 0.25, 0.25, -0.25}
        };
        hardwareMap.setActive(false);

        shooterFlyWheel.getTransforms().add(new Rotate(0, getCenterPivotX(shooterFlyWheel), getCenterPivotY(shooterFlyWheel)));
        shooterIndexer.getTransforms().add(new Rotate(0, getCenterPivotX(shooterIndexer), getEdgePivotY(shooterIndexer)));
        intake1.getTransforms().add(new Rotate(0, getCenterPivotX(intake1), getCenterPivotY(intake1)));
        intake2.getTransforms().add(new Rotate(0, getCenterPivotX(intake2), getCenterPivotY(intake2)));

        /*
        Scales the arm with pivot point at center of back of robot (which corresponds to the back of the arm).
        The Y-scaling is initialized to 1.0 (i.e., arm fully retracted)
        We will never change the X-scaling (we don't need the arm to get fatter, only longer)
         */
        armScaleTransform = new Scale(1.0, 1.0, getCenterPivotX(arm), getEdgePivotY(arm));
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

    private double getCenterPivotX(Rectangle r) {
        return r.getX() + r.getWidth() / 2;
    }

    private double getCenterPivotY(Rectangle r) {
        return r.getY() + r.getHeight() / 2;
    }

    private double getEdgePivotY(Rectangle r) {
        return r.getY() + r.getHeight();
    }

    protected void createHardwareMap() {
        hardwareMap = new HardwareMap();
        String[] motorNames = new String[]{"blMotor", "flMotor", "frMotor", "brMotor"};
        for (String name : motorNames) {
            hardwareMap.put(name, new DcMotorExImpl(MOTOR_TYPE));
        }
        String[] distNames = new String[]{"front_distance", "left_distance", "back_distance", "right_distance"};
        for (String name : distNames) {
            hardwareMap.put(name, controller.new DistanceSensorImpl());
        }
        hardwareMap.put("imu", new BNO055IMUImpl(this, 10));
        hardwareMap.put("wheelMotor", new DcMotorExImpl(SHOOTER_MOTOR_TYPE));
        hardwareMap.put("indexServo", new ServoImpl());
        hardwareMap.put("intakeMotor", new DcMotorExImpl(INTAKE_MOTOR_TYPE));
        hardwareMap.put("wobbleGrabberArm", new DcMotorExImpl(ARM_MOTOR_TYPE));
        hardwareMap.put("wobbleGrabberClaw", new ServoImpl());
        hardwareMap.put("aimServo", new ServoImpl());
        hardwareMap.put("elevatorServo", new CRServoImpl(180));
        hardwareMap.put("topSensor", controller.new DistanceSensorImpl());
        hardwareMap.put("bottomSensor", controller.new DistanceSensorImpl());
        hardwareMap.put("intakeSensor", controller.new DistanceSensorImpl());
        hardwareMap.put("ringDetector", controller.new DistanceSensorImpl());
        hardwareMap.put("intakeServo", new ServoImpl());
        hardwareMap.put("armSwitch", new DigitalChannelImpl());
        hardwareMap.put("intakeLEDs", new RevBlinkinLedDriver());
        hardwareMap.put("ringCountDisplay", new RevBlinkinLedDriver());
    }

    public synchronized void updateStateAndSensors(double millis) {
        double[] deltaPos = new double[4];
        double[] w = new double[4];

        for (int i = 0; i < 4; i++) {
            deltaPos[i] = -wheelMotors[i].update(millis);
            w[i] = deltaPos[i] * wheelCircumference / MOTOR_TYPE.TICKS_PER_ROTATION;
            if (i < 2) w[i] = -w[i];
        }

        double[] robotDeltaPos = new double[]{0, 0, 0, 0};
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                robotDeltaPos[i] += tWR[i][j] * w[j];
            }
        }

        double dxR = robotDeltaPos[0];
        double dyR = robotDeltaPos[1];
        double headingChange = robotDeltaPos[2];
        double avgHeading = headingRadians + headingChange / 2.0;

        double sin = Math.sin(avgHeading);
        double cos = Math.cos(avgHeading);

        double x0 = x;
        double y0 = y;

        x += dxR * cos - dyR * sin;
        y += dxR * sin + dyR * cos;
        headingRadians += headingChange;

        if (headingRadians > Math.PI) {
            headingRadians -= 2.0 * Math.PI;
        } else if (headingRadians < -Math.PI) {
            headingRadians += 2.0 * Math.PI;
        }

        constrainToBoundaries();

        if (y != y0 || x != x0) {
            directionOfTravel = Math.atan2(y - y0, x - x0);
        } else {
            directionOfTravel = null;
        }

        imu.updateHeadingRadians(-headingRadians);

        final double piOver2 = Math.PI / 2.0;
        for (int i = 0; i < 4; i++) {
            double sensorHeading = AngleUtils.normalizeRadians(headingRadians + i * piOver2);
            distanceSensors[i].updateDistance(x - halfBotWidth * Math.sin(sensorHeading),
                    y + halfBotWidth * Math.cos(sensorHeading), sensorHeading);
        }

        shooterMotor.update(millis);
        intakeMotor.update(millis);

        /*
        Calculate the new value of armScale based on interval motion of the arm motor. BUT, do not manipulate
        the arm UI here. Do that in the updateDisplay method, which will ultimately be called from the UI Thread.
         */
        armMotor.update(millis);
        armScale = -Math.cos(armMotor.getCurrentPosition() / ARM_MOTOR_TYPE.TICKS_PER_ROTATION * Math.PI * 2.0);

        double pixelsPerInch = controller.getField().getPixelsPerInch();

        if (ringInIntake != null) {
//            double timeInIntakeMillis = System.currentTimeMillis() - intakeTimeMillis;
            positionInIntake += intakeMotor.getPower() * (millis / INTAKE_TO_HOPPER_TIME_MILLIS);
            if (positionInIntake <= 1 && positionInIntake > 0) {
                double t = positionInIntake;
                // calculate the intake location
                double ix = INTAKE_R_INCHES * pixelsPerInch * Math.cos(headingRadians + INTAKE_ANGLE_DEG * Math.PI / 180.0);
                double iy = INTAKE_R_INCHES * pixelsPerInch * Math.sin(headingRadians + INTAKE_ANGLE_DEG * Math.PI / 180.0);
                Vector2D u = new Vector2D(ix, iy);
                // calculate the hopper location
                double hx = HOPPER_R_INCHES * pixelsPerInch * Math.cos(headingRadians + HOPPER_ANGLE_DEG * Math.PI / 180.0);
                double hy = HOPPER_R_INCHES * pixelsPerInch * Math.sin(headingRadians + HOPPER_ANGLE_DEG * Math.PI / 180.0);
                Vector2D v = new Vector2D(hx, hy);
                // interpolate the position of the ring within the intake based on time elapsed
                Vector2D p = u.added(v.subtracted(u).multiplied(t));
                ringInIntake.setLocation(x + p.x, y + p.y);
            }
            else if (positionInIntake > 1) {//ring is intaken
                ringsInHopper.add(ringInIntake);
                ringInIntake = null;
                positionInIntake = 0;
            }
            else {//ring is spat out
                ringInIntake.setControlledBy(null);
                ringInIntake = null;
                positionInIntake = 0;
            }
        }

        for (Ring r : ringsInHopper) {
            // set ring location to the current location of the hopper given the robot's location and heading
            double dx = HOPPER_R_INCHES * pixelsPerInch * Math.cos(headingRadians + HOPPER_ANGLE_DEG * Math.PI / 180.0);
            double dy = HOPPER_R_INCHES * pixelsPerInch * Math.sin(headingRadians + HOPPER_ANGLE_DEG * Math.PI / 180.0);
            r.setLocation(x + dx, y + dy);
        }

        if (wobble_goal != null) {
            if (handServo.getPosition() >= 0.75 || Math.abs(armMotor.getCurrentPosition()) / ARM_MOTOR_TYPE.TICKS_PER_ROTATION <= 0.25) {
                // when controlled, the wobble goal will stay between the fingers
                Vector2D p = getHandLocation();
                wobble_goal.setLocation(x + p.x, y + p.y);
            }
            else {
                // release the wobble goal
                wobble_goal.setControlledBy(null);
                wobble_goal = null;
            }
        }

        // shoot ring
        if (Math.abs(shooterMotor.getPower()) > 0.001) {
            if (readyToShoot && shooterServo.getPosition() >= 0.9 && ringsInHopper.size() > 0) {
                Ring r = ringsInHopper.remove(0);
                r.setControlledBy(null);
                double dx = SHOOTER_EXIT_R_INCHES * pixelsPerInch * Math.cos(headingRadians + SHOOTER_EXIT_ANGLE * Math.PI / 180.0);
                double dy = SHOOTER_EXIT_R_INCHES * pixelsPerInch * Math.sin(headingRadians + SHOOTER_EXIT_ANGLE * Math.PI / 180.0);
                r.setLocation(x + dx, y + dy);
                double shooterInchesPerSecond = SHOOTER_MAX_INCHES_PER_SEC * Math.abs(shooterMotor.getPower());
                double vx = shooterInchesPerSecond * pixelsPerInch * Math.cos(headingRadians + SHOOTER_OFFSET_DEG * Math.PI / 180.0);
                double vy = shooterInchesPerSecond * pixelsPerInch * Math.sin(headingRadians + SHOOTER_OFFSET_DEG * Math.PI / 180.0);
                r.setVelocity(vx, vy);
                r.setInFlight(true);
                readyToShoot = false;
            }
        }

        if (!readyToShoot && shooterServo.getPosition() <= 0.1) {
            readyToShoot = true;
        }
        
        if (ringInIntake == null)
            intakeSensor.setDistance(200);
        else
            intakeSensor.setDistance(100);
    }

    private Vector2D getHandLocation() {
        double dx = handTranslateTransform.getX();
        double dy = 75 - leftFingerTranslateTransform.getY(); // TODO Fix this - it's not quite right
        return new Vector2D(dx, dy).rotated(headingRadians);
    }

    public void interact(VirtualGameElement e) {
        if (e instanceof Ring) {
            Ring r = (Ring) e;
            if (r.isOnField() && !r.isInFlight() && r.getControlledBy() == null) {
                if (r.isStationary() && directionOfTravel != null) {
                    // ring is stationary and robot is moving
                    double diff = headingRadians + Math.PI / 2.0 - directionOfTravel;
                    double deltaAngle = Math.atan2(Math.sin(diff), Math.cos(diff));
                    if (intakeMotor.getPower() > 0 && Math.toDegrees(Math.abs(deltaAngle)) <= INTAKE_DIR_TOLERANCE_DEG) {
                        // intake is on and direction of travel is aligned with the intake
                        double dx = r.getX() - this.x;
                        double dy = r.getY() - this.y;
                        double directionToRing = Math.atan2(dy, dx);
                        if (Math.abs(directionToRing - directionOfTravel) <= Math.toRadians(INTAKE_ANGLE_TOLERANCE_DEG)) {
                            // ring is in front of the running intake
                            double distanceToRingInches = Math.sqrt(dx * dx + dy * dy) / (controller.getField().fieldWidth / 144);
                            if (distanceToRingInches <= INTAKE_DIST_TOLERANCE_INCHES) {
                                // ring is close enough to be picked up and there is not another ring in the intake
                                if (ringInIntake == null) {
                                    // take control of the ring
                                    r.setControlledBy(this);
                                    ringInIntake = r;
                                    intakeTimeMillis = System.currentTimeMillis();
                                }
                            }
                        }
                    }

                    // if ring is not controlled at this point, then push it away from the robot
                    if (r.getControlledBy() == null) {
                        collide(r, Ring.RING_RADIUS_INCHES);
                    }
                } else {
                    // if the ring is in motion and collides with the robot, then stop the ring
                    if (r.getControlledBy() == null) {
                        if (collide(r, Ring.RING_RADIUS_INCHES)) {
                            r.setVelocity(0.0, 0.0);
                        }
                    }
                }
            }
        }
        else if (e instanceof WobbleGoal) {
            if (e.getControlledBy() == null) {
                WobbleGoal w = (WobbleGoal) e;
                if (Math.abs(armMotor.getCurrentPosition()) / ARM_MOTOR_TYPE.TICKS_PER_ROTATION >= WOBBLE_GRAB_TURN_FRACTION) {
                    // arm is in position to grab wobble goal
                    if (handServo.getPosition() >= 0.75) {
                        // hand is grasping
                        // calculate distance from wobble goal to center of gripper
                        Vector2D h = getHandLocation();
                        double dist = h.added(new Vector2D(x, y)).subtracted(w.getLocation()).length();
                        if (dist <= 10) {
                            w.setControlledBy(this);
                            wobble_goal = w;
                        }
                    }
                }
                collide(e, WobbleGoal.RADIUS_INCHES);
            }
        }
    }

    private boolean collide(VirtualGameElement r, double radiusInches) {
        Vector2D[] robotBoundary = getBoundary();
        Vector2D point = new Vector2D(r.getX(), r.getY());

        boolean collision = false;

        double pixelsPerInch = controller.getField().getPixelsPerInch();
        for (int i = 0; i < robotBoundary.length; ++i) {
            int j = (i == 0) ? 3 : i - 1;
            // calculate distance from ring to boundary segment of the robot
            double dist = distanceToSegment(robotBoundary[j], robotBoundary[i], point);
            // calculate the amount of overlap
            double overlapPixels = Math.max(0.0, (radiusInches * pixelsPerInch) - dist);
            if (overlapPixels > 0.0) {
                // move the ring coordinates to eliminate the overlap
                Vector2D u = normalToSegment(robotBoundary[j], robotBoundary[i]);
                r.setLocation(point.added(u.multiplied(overlapPixels)));
                VirtualField field = controller.getField();
                double radius = radiusInches * field.getPixelsPerInch();
                double rx = Math.max(field.X_MIN + radius, Math.min(r.getX(), field.X_MAX - radius));
                double ry = Math.max(field.Y_MIN + radius, Math.min(r.getY(), field.Y_MAX - radius));
                r.setLocation(rx, ry);
                collision = true;
            }
        }

        return collision;
    }

    private Vector2D[] getBoundary() {
        // initialize boundary points of standard 18"x18" robot
        Vector2D[] boundary = new Vector2D[]{
                new Vector2D(-9.0, 9.0), // left-top
                new Vector2D(9.0, 9.0), // right-top
                new Vector2D(9.0, -9.0), // right-bottom
                new Vector2D(-9.0, -9.0) // left-bottom
        };

        double pixelsPerInch = controller.getField().getPixelsPerInch();
        Vector2D location = new Vector2D(x, y);

        for (Vector2D v : boundary) {
            // scale points to pixels(
            v.multiply(pixelsPerInch);

            // rotate by heading
            v.rotate(headingRadians);

            // translate by x, y
            v.add(location);
        }

        return boundary;
    }

    /**
     * Return the minimum distance between a line segment and a point.
     * @param s1 endpoint of segment
     * @param s2 endpoint of segment
     * @param point point
     * @return closest distance from segment to point, perpendicular or to endpoint
     * @implNote https://stackoverflow.com/a/1501725
     */
    private double distanceToSegment(Vector2D s1, Vector2D s2, Vector2D point) {
        // consider the line extending the segment, parameterized as s1 + t * (s2 - s1)
        Vector2D v = s2.subtracted(s1); // v = s2 - s1
        double l2 = v.dot(v);  // squared length of the segment
        // find projection of point onto the line containing the segment and
        // clamp to the interval [0, 1], where t=0 means s1 and t=1 is s2
        double t = Math.max(0.0, Math.min(1.0, point.subtracted(s1).dot(v) / l2));
        // calculate the point on the line segment corresponding to t
        Vector2D p = s1.added(v.multiplied(t));
        // calculate the distance
        return point.subtracted(p).length();
    }

    /**
     * Return a unit vector normal to the line segment [s1, s2] formed by taking the vector
     * s2 - s1 and rotating it counterclockwise PI/2 radians (90 deg) and normalizing it.
     * @param s1 segment endpoint
     * @param s2 segment endpoint
     * @return unit vector normal to the segment
     */
    private Vector2D normalToSegment(Vector2D s1, Vector2D s2) {
        return s2.subtracted(s1).rotated(Math.PI / 2.0).normalized();
    }

    public synchronized void updateDisplay() {
        super.updateDisplay();
        ((Rotate) shooterFlyWheel.getTransforms().get(0)).setAngle(-shooterMotor.getCurrentPosition() / 14.0 * Math.PI); // the actual angle isn't important as long as it spins
        ((Rotate) shooterIndexer.getTransforms().get(0)).setAngle(-shooterServo.getPosition() * 180.0);
        ((Rotate) intake1.getTransforms().get(0)).setAngle(intakeMotor.getCurrentPosition() / 5.0); // the actual angle isn't important
        ((Rotate) intake2.getTransforms().get(0)).setAngle(-intakeMotor.getCurrentPosition() / 5.0); // the actual angle isn't important

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
        double fingerPos = 7.5 * handServo.getInternalPosition();
        if (Math.abs(fingerPos - leftFingerTranslateTransform.getX()) > 0.001) {
            leftFingerTranslateTransform.setX(fingerPos);
            rightFingerTranslateTransform.setX(-fingerPos);
        }
    
        intakeSensor.updateDistance(0, 200 - ringWidth, 0);
    }

    public void powerDownAndReset() {
        for (int i = 0; i < 4; i++) {
            wheelMotors[i].stopAndReset();
        }
        shooterMotor.stopAndReset();
        intakeMotor.stopAndReset();
        imu.close();
    }

    @Override
    public void initControl(List<VirtualGameElement> gameElements) {
        // release control of old game elements on reinitialization
        if (wobble_goal != null) {
            wobble_goal.setControlledBy(null);
        }
        wobble_goal = null;
        for (Ring r : ringsInHopper) {
            r.setControlledBy(null);
        }
        ringsInHopper.clear();

        // preload wobble goal 1
        for (VirtualGameElement e : gameElements) {
            if (e instanceof WobbleGoal) {
                wobble_goal = (WobbleGoal) e;
                wobble_goal.setControlledBy(this);
                break;
            }
        }

        // preload 3 rings
        for (VirtualGameElement e : gameElements) {
            if (e instanceof Ring) {
                Ring r = (Ring) e;
                r.setControlledBy(this);
                ringsInHopper.add(r);
                if (ringsInHopper.size() == 3) {
                    break;
                }
            }
        }
    }
}
