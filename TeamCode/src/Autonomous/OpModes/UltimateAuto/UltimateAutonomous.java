package Autonomous.OpModes.UltimateAuto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import Actions.Ultimate.RingIntakeSystemV1;
import Actions.Ultimate.ShooterSystemV1;
import Actions.Ultimate.WobbleGrabberV1;
import Autonomous.AutoAlliance;
import Autonomous.Location;
import Autonomous.RingDetector;
import DriveEngine.Ultimate.UltimateNavigation;
import SensorHandlers.LIDARSensor;

import static Autonomous.ConfigVariables.PARKING_LOCATION;
import static Autonomous.ConfigVariables.POWER_SHOT_LEFT;
import static Autonomous.ConfigVariables.POWER_SHOT_MIDDLE;
import static Autonomous.ConfigVariables.POWER_SHOT_MIDDLE_ON_LINE;
import static Autonomous.ConfigVariables.POWER_SHOT_RIGHT;
import static Autonomous.ConfigVariables.RED_WOBBLE_GOAL_LEFT;
import static Autonomous.ConfigVariables.RED_WOBBLE_GOAL_LEFT_CHECKPOINT;
import static Autonomous.ConfigVariables.RED_ZONE_ONE;
import static Autonomous.ConfigVariables.RED_ZONE_THREE;
import static Autonomous.ConfigVariables.RED_ZONE_TWO;
import static Autonomous.ConfigVariables.RING_DETECTION_POINT;
import static Autonomous.ConfigVariables.SHOOTING_LINE_POINT;

/**
 * Author: Ethan Fisher
 * Date: 10/29/2020
 *
 * Autonomous for Ultimate Goal
 */
public class UltimateAutonomous {

    private final AutoAlliance alliance;
    private final LinearOpMode mode;

    public UltimateNavigation robot;
    private double startHeading;
    private RingDetector ringDetector;

    private WobbleGrabberV1 wobbleGrabber;
    private ShooterSystemV1 shooter;
    private RingIntakeSystemV1 intake;
    private LIDARSensor topSensor, bottomSensor;

    private static final double MAX_SPEED = UltimateNavigation.MAX_SPEED;

    public UltimateAutonomous(AutoAlliance alliance, Location startLocation, final LinearOpMode mode) {

        this.alliance = alliance;
        this.mode = mode;
        startHeading = startLocation.getHeading();

//        VuforiaHelper vuforia = new VuforiaHelper(mode.hardwareMap);
//        ringDetector = ColorDetector.ringDetector(vuforia);

        wobbleGrabber = new WobbleGrabberV1(mode.hardwareMap);
        shooter = new ShooterSystemV1(mode.hardwareMap, mode);
        intake = new RingIntakeSystemV1(mode.hardwareMap);

        robot = new UltimateNavigation(mode.hardwareMap, redToBlue(startLocation), "RobotConfig/UltimateV1.json");
        topSensor = new LIDARSensor(mode.hardwareMap.get(DistanceSensor.class, "topSensor"), "topSensor");
        bottomSensor = new LIDARSensor(mode.hardwareMap.get(DistanceSensor.class, "bottomSensor"), "bottomSensor");

        ringDetector = new RingDetector(topSensor, bottomSensor);
    }

    public void driveToLeftWobbleGoalAndGrab() {
        driveToLocationOnHeading(RED_WOBBLE_GOAL_LEFT_CHECKPOINT, 0);

        wobbleGrabber.releaseWobbleGoal();
        wobbleGrabber.lowerArm();

        driveToLocationOnHeading(RED_WOBBLE_GOAL_LEFT, 0);

        // drive right a little bit maybe
        wobbleGrabber.grabWobbleGoal();
        sleep(600);
        wobbleGrabber.liftArm();
        waitForArm();
    }

//    public void driveToRightWobbleGoal() {
//        robot.turnToHeading(UltimateNavigation.WEST, mode);
//        Location target = new Location(RED_WOBBLE_GOAL_RIGHT);
//        target.setHeading(robot.orientation.getOrientation());
//        driveToLocation(target);
//    }

//    public void driveToRightStartingPos() {
//        Location target = new Location(STARTING_ROBOT_LOCATION_RIGHT);
//        target.setHeading(robot.orientation.getOrientation());
//        driveToLocation(target);
//    }

    public Location getZone(int numRings) {
        if (numRings == 0)
            return RED_ZONE_ONE;
        else if (numRings == 1)
            return RED_ZONE_TWO;
        else
            return RED_ZONE_THREE;
    }

    public void driveToZone(int numRings) {
        driveToLocationOnInitHeading(getZone(numRings));
    }

    public void driveToRingCheckpoint() {
        driveToLocationOnInitHeading(RING_DETECTION_POINT);
    }

    // TODO we can't use the robot location and redToBlue if we're on the blue side, since it will
    // think we're going to the other side of the field
    public void moveToShootLocation() {
//        driveToLocation(new Location(robot.getRobotLocation().getX(), SHOOTING_LINE_POINT.getY(), UltimateNavigation.SOUTH));
        driveToLocation(SHOOTING_LINE_POINT);
        sleep(500);
    }

//    public void moveBehindShootLine() {
//        double zoneDistFromLine = (wobbleZone.getY() - SHOOT_LINE.getY()) / 2.54;
//        double distToDrive = zoneDistFromLine + 20;
//        robot.driveDistance(distToDrive, UltimateNavigation.SOUTH, MAX_SPEED, mode);
//        robot.brake();
//        sleep(1000);
//    }

    public void shootPowerShots() {
        driveToLocationOnInitHeading(POWER_SHOT_MIDDLE_ON_LINE);
        turnToZero();

        shooter.turnOnShooterWheel();
        shooter.setShooter(ShooterSystemV1.POWER_SHOT_POSITION);

        robot.turnToShoot(POWER_SHOT_LEFT, mode);

        sleep(1000);
        shooter.shoot();
        sleep(1000);
        shooter.shoot();
        sleep(1000);

        robot.turnToShoot(POWER_SHOT_MIDDLE, mode);

//        driveToLocationOnHeading(POWER_SHOT_MIDDLE_ON_LINE, UltimateNavigation.NORTH);
        shooter.shoot();
        sleep(1000);
        shooter.shoot();
        sleep(1000);

        robot.turnToShoot(POWER_SHOT_RIGHT, mode);

//        driveToLocationOnHeading(POWER_SHOT_RIGHT_ON_LINE, UltimateNavigation.NORTH);
        shooter.shoot();
        sleep(1000);
        shooter.shoot();
        sleep(1000);

        mode.telemetry.update();

        // TODO turn instead of moving position

        shooter.turnOffShooterWheel();
        shooter.setShooter(ShooterSystemV1.LOWERED_POSITION);
    }

//    public void driveToStartingRings() {
//        // drive to the heading, within a certain range
//        // TODO maybe move in the x and then in the y for more accuracy???
//        robot.turnToHeading(UltimateNavigation.SOUTH, mode);
//        double desiredDistanceFromRings = 2;
//        robot.driveToLocationPID(STARTING_RING_PILE, MAX_SPEED, desiredDistanceFromRings, mode);
//        sleep(1000);
//    }

//    public void grabStartingPileRings() {
//        // turn towards the rings, then pick them up
//        intake.turnOn();
//        driveDistance(10, UltimateNavigation.SOUTH);
//        robot.brake();
//        sleep(1000);
//        intake.turnOff();
//    }

    public void waitForArm() { while(mode.opModeIsActive() && wobbleGrabber.armIsBusy()); }

    public void dropWobbleGoal() {
        wobbleGrabber.lowerArm();
        waitForArm();
        wobbleGrabber.releaseWobbleGoal();
        sleep(200);
        wobbleGrabber.raiseToVertical();
        waitForArm();
        wobbleGrabber.grabWobbleGoal();
    }

    public void pickupWobbleGoal() {
        wobbleGrabber.lowerArm();
        waitForArm();
        wobbleGrabber.grabWobbleGoal();
        sleep(600);
        wobbleGrabber.raiseToVertical();
        waitForArm();
    }

    public void shootThreeRings() {
        shooter.setShooter(ShooterSystemV1.HIGHEST_POSITION);
        shooter.turnOnShooterWheel();
        sleep(1000);
//        shooter.keepElevatorAtTop();
        for  (int i = 0; i <= 3; i++) {
//            shooter.keepElevatorAtTop();
            shooter.shoot(); // Index ring into shooter
            sleep(500); // Wait for index
            shooter.shoot(); // Retract indexer
            sleep(750); // Wait for retract
        }
        shooter.turnOffShooterWheel();
        shooter.setShooter(ShooterSystemV1.LOWERED_POSITION);
        shooter.lowerElevator();
    }

    public void grabStartingPileRings() {
        turnToInitHeading();
        intake.intakeOn();

        driveDistance(20, UltimateNavigation.NORTH);
        mode.sleep(2000);
        robot.brake();

        intake.intakeOff();
        turnToInitHeading();
    }

    // converts red to blue. If it is red, nothing happens
    public Location redToBlue(Location location) {
        if (alliance == AutoAlliance.BLUE)
            return new Location(-location.getX(), location.getY(), 360 - location.getHeading());
        return location;
    }

    public int detectNumRings() { return ringDetector.getNumRings(); }

    public void stop() {
        shooter.stop();
        robot.stopNavigation();
        topSensor.kill();
        bottomSensor.kill();
        ringDetector.kill();
        mode.telemetry.addData("Robot", "Stopped");
        mode.telemetry.update();
    }
    public void sleep(long milliseconds) { mode.sleep(milliseconds); }
    public void park() { robot.driveDistance(PARKING_LOCATION.getY() - robot.getRobotLocation().getY(), UltimateNavigation.NORTH, MAX_SPEED, mode); }

    public void turnToZero() { robot.turnToHeading(UltimateNavigation.NORTH, mode); }
    public void turnToInitHeading() { robot.turnToHeading(startHeading, mode); }
    public void turnToHeading(double heading) { robot.turnToHeading(heading, mode); }

//    public void driveToZoneWaypoint() { driveToLocationOnInitHeading(ZONE_WAYPOINT); }
    public void driveToLocationOnInitHeading(Location location) {
        driveToLocationOnHeading(location, startHeading);
    }
    public void driveToLocationOnHeading(Location location, double heading) {
        location.setHeading(heading);
        driveToLocation(location);
    }
    
//    public void dropDownIntake() {todo use intake system v2 before uncommenting this
//        intake.dropDown();
//    }
    
    public void driveToLocation(Location location) { robot.driveToLocationPID(redToBlue(location), MAX_SPEED, mode); }
    public void driveDistance(double distanceInInches, double heading) { robot.driveDistance(distanceInInches, heading, MAX_SPEED, mode); }
    public WobbleGrabberV1 getWobbleGrabber() { return wobbleGrabber; }
    public ShooterSystemV1 getShooter() { return shooter; }
    public RingIntakeSystemV1 getIntake() { return intake; }
}