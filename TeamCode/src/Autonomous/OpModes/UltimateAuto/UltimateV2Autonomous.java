package Autonomous.OpModes.UltimateAuto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import Actions.Ultimate.RingIntakeSystemV2Test;
import Actions.Ultimate.ShooterSystemV2Test;
import Actions.Ultimate.WobbleGrabberV2Test;
import Autonomous.AutoAlliance;
import Autonomous.Location;
import Autonomous.RingCount;
import Autonomous.VisionHelperUltimateGoal;
import DriveEngine.Ultimate.UltimateNavigation2;

import static Autonomous.ConfigVariables.LEFT_POWER_SHOT_HEADING;
import static Autonomous.ConfigVariables.MIDDLE_POWER_SHOT_HEADING;
import static Autonomous.ConfigVariables.PARKING_LOCATION;
import static Autonomous.ConfigVariables.QUAD_STACK_END_POINT;
import static Autonomous.ConfigVariables.RED_WOBBLE_GOAL_LEFT;
import static Autonomous.ConfigVariables.RED_ZONE_ONE;
import static Autonomous.ConfigVariables.RED_ZONE_THREE;
import static Autonomous.ConfigVariables.RED_ZONE_TWO;
import static Autonomous.ConfigVariables.RIGHT_POWER_SHOT_HEADING;
import static Autonomous.ConfigVariables.RING_PATH_TURN;
import static Autonomous.ConfigVariables.RING_STACK_START_POINT;
import static Autonomous.ConfigVariables.SHOOTING_LINE_POINT;

/**
 * Author: Ethan Fisher
 * Date: 10/29/2020
 *
 * Autonomous for Ultimate Goal
 */
public class UltimateV2Autonomous {

    private final AutoAlliance alliance;
    private final LinearOpMode mode;

    protected UltimateNavigation2 robot;

    protected WobbleGrabberV2Test wobbleGrabber;
    protected ShooterSystemV2Test shooter;
    protected RingIntakeSystemV2Test intake;

    protected VisionHelperUltimateGoal vision;

    protected static final int WOBBLE_OFFSET = 4;
    protected static final double MAX_SPEED = 50;
    protected static final double MED_SPEED = 25;
    protected static final double LOW_SPEED = 15;
    protected static final double MIN_SPEED = 5;
    protected static final long SLEEP_TIME = 100;

    public UltimateV2Autonomous(AutoAlliance alliance, Location startLocation, final LinearOpMode mode) {

        this.alliance = alliance;
        this.mode = mode;

        wobbleGrabber = new WobbleGrabberV2Test(mode.hardwareMap);
        shooter = new ShooterSystemV2Test(mode.hardwareMap);
        intake = new RingIntakeSystemV2Test(mode.hardwareMap);

        startLocation = redToBlue(startLocation);
        try {
            robot = new UltimateNavigation2(mode.hardwareMap, startLocation, startLocation.getHeading(), "RobotConfig/UltimateV2.json");
        } catch (Exception e) {
            e.printStackTrace();
        }

        vision = new VisionHelperUltimateGoal(VisionHelperUltimateGoal.WEBCAM, mode.hardwareMap);
    }

    // converts red to blue. If it is red, nothing happens
    public Location redToBlue(Location location) {
        if (alliance == AutoAlliance.BLUE)
            return new Location(-location.getX(), location.getY(), 360 - location.getHeading());
        return location;
    }

    public void kill() {
        vision.kill();
        wobbleGrabber.kill();
        shooter.kill();
        intake.kill();
        robot.stopNavigation();

        mode.telemetry.addData("Robot", "Stopped");
        mode.telemetry.update();
    }

    // AUTONOMOUS FUNCTIONS HERE
    protected void init() {
        wobbleGrabber.setClawGrabAngle();
        shooter.setIndexLeft();
        intake.intakeServoIn();
        // set initial servo positions
    }

    // drives from current location to where power shots must be performed
    // performs power shots from right to left
    protected void performPowerShots(LinearOpMode mode, double runtime) {
        if(mode.opModeIsActive() && 30 - runtime > 10) { // if the time remaining is more than the required action time, perform it
                shooter.spinUp();
                robot.driveDistanceToLocation(SHOOTING_LINE_POINT, LOW_SPEED, mode);

                // perform shots
                shooter.setPowerShotPower(); // spin up motor to expected power shot rpm
                robot.turnToHeading(RIGHT_POWER_SHOT_HEADING, mode); // turn to heading for first power shot and shoot
                indexShooter();

                robot.turnToHeading(MIDDLE_POWER_SHOT_HEADING, mode); // turn to heading for second power shot and shoot
                indexShooter();

                robot.turnToHeading(LEFT_POWER_SHOT_HEADING, mode); // turn to heading for third power shot and shoot
                indexShooter();
        }
    }

    // drives to the correct wobble goal delivery zone from the current robot location
    protected void deliverWobbleGoal(LinearOpMode mode, RingCount ringCount, double runtime, int wobbleNum) {
        if(mode.opModeIsActive() && 30 - runtime > 10) {
                Location targetLocation = RED_ZONE_ONE;
                switch (ringCount) {
                    case NO_RINGS:
                        break; // if no rings, zone one
                    case SINGLE_STACK:
                        targetLocation = RED_ZONE_TWO; // if one ring, zone two
                        break;
                    case QUAD_STACK:
                        targetLocation = RED_ZONE_THREE; // if four rings, zone three
                        break;
                }
                if(wobbleNum == 1) {
                    robot.driveToLocationPID(targetLocation, MED_SPEED, mode);
                }
                else { // Place second wobble goal slightly off from the location of the first to avoid collision
                    Location offsetTarget = new Location(targetLocation.getX() - WOBBLE_OFFSET, targetLocation.getY() - WOBBLE_OFFSET);
                    robot.driveToLocationPID(offsetTarget, MED_SPEED, mode);
                }

                wobbleGrabber.setGrabAndDropAngle();
                wobbleGrabber.releaseWobble();
                mode.sleep(SLEEP_TIME);
                wobbleGrabber.setLiftAngle();
            }

    }

    // reorients and drives to intake the extra rings but only if there are extra rings
    protected void intakeExtraRings(LinearOpMode mode, RingCount ringCount, double runtime) {
        if(mode.opModeIsActive() && 30 - runtime > 10 && ringCount != RingCount.NO_RINGS) {
                robot.driveToLocationPID(RING_PATH_TURN, MED_SPEED, mode);
                robot.turnToHeading(UltimateNavigation2.EAST, 5, mode);
                intake.intakeOn();
                intake.update();
                robot.driveToLocationPID(RING_STACK_START_POINT, MED_SPEED, mode); // consider using drive to location PID -- is thought to be more accurate?
                // do a funky intake thingy here
                if(ringCount == ringCount.SINGLE_STACK) {
                    robot.driveDistance(12, robot.getOrientation(), MED_SPEED, mode);
                }
                else {
                    robot.driveToLocationPID(QUAD_STACK_END_POINT, LOW_SPEED, mode);
                }
            }
        }

    // drives to the second wobble goal and grabs it
    protected void obtainSecondWobbleGoal(LinearOpMode mode, double runtime) {
        if(mode.opModeIsActive() /*&& 30 - runtime > 10*/) {
            robot.turnToHeading(UltimateNavigation2.EAST, mode);
            robot.driveToLocationPID(RED_WOBBLE_GOAL_LEFT, MED_SPEED, mode);
            wobbleGrabber.setGrabAndDropAngle(); // Bring arm down to grab angle
            wobbleGrabber.releaseWobble(); // Open claw
            robot.driveOnHeading(robot.getOrientation(), LOW_SPEED); // Drive forwards, slowly
            while(!wobbleGrabber.sensor.isPressed() && mode.opModeIsActive());
            robot.brake(); // Once sensor is pressed, stop
            wobbleGrabber.setClawGrabAngle(); // Close claw
            mode.sleep(SLEEP_TIME);
            wobbleGrabber.setLiftAngle(); // Lift arm back up
            robot.turnToHeading(UltimateNavigation2.NORTH, mode);
        }
    }

    // drives to the correct position to shoot the extra rings then shoots them if there are extra rings
    protected void shootExtraRings(LinearOpMode mode, RingCount ringCount, double runtime) {
        if(mode.opModeIsActive() && 30 - runtime > 5 && ringCount != RingCount.NO_RINGS) {
                robot.turnToHeading(UltimateNavigation2.NORTH, 1, mode);
                robot.driveDistanceToLocation(SHOOTING_LINE_POINT, MED_SPEED, mode);

                // shoot rings
                shooter.setHighGoalPower(); // spin up motor to proper high goal rpm
                if(ringCount == RingCount.SINGLE_STACK){ // if there is only one extra ring, only index once
                    indexShooter();
                }
                else { // otherwise (there are four rings), index three times
                    for(int i = 0; i < 3; i++){
                        indexShooter();
                    }
            }
        }
    }

    protected void indexShooter(){
        shooter.setIndexLeft();
        mode.sleep(SLEEP_TIME);
        shooter.setIndexRight();
        mode.sleep(SLEEP_TIME);
    }

    protected void funkyIntakeThingy(){
        // TODO figure out how to intake 3/4 rings in starting stack

    }

    protected void dropIntakeAndWobble(LinearOpMode mode) {
        if(mode.opModeIsActive()){
            intake.intakeServoOut();
            mode.sleep(SLEEP_TIME);
            intake.intakeServoIn();
            wobbleGrabber.setLiftAngle();
        }
    }


    // parks the robot over the launch line
    protected void park(LinearOpMode mode) {
        // turn everything off
        if(mode.opModeIsActive()){
            wobbleGrabber.pause();
            shooter.pauseShooter();
            intake.intakeOff();

            // drive to parking line
            robot.driveDistanceToLocation(PARKING_LOCATION, MAX_SPEED, mode);
        }

    }
}