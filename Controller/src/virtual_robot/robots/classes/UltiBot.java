package virtual_robot.robots.classes;

import com.qualcomm.robotcore.hardware.DcMotorExImpl;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.ServoImpl;
import com.qualcomm.robotcore.hardware.configuration.MotorType;
import com.qualcomm.robotcore.util.ElapsedTime;
import javafx.fxml.FXML;
import javafx.scene.Group;
import javafx.scene.transform.Translate;
import org.dyn4j.dynamics.Body;
import org.dyn4j.dynamics.BodyFixture;
import org.dyn4j.geometry.Rectangle;
import org.dyn4j.geometry.Vector2;
import org.dyn4j.world.NarrowphaseCollisionData;
import org.dyn4j.world.listener.CollisionListenerAdapter;
import virtual_robot.controller.Game;
import virtual_robot.games.UltimateGoal;
import virtual_robot.controller.BotConfig;
import virtual_robot.controller.Filters;
import virtual_robot.controller.VirtualField;
import virtual_robot.game_elements.classes.Ring;
import virtual_robot.robots.ControlsElements;
import virtual_robot.dyn4j.Dyn4jUtil;
import virtual_robot.dyn4j.FixtureData;
import virtual_robot.dyn4j.Slide;

import java.util.ArrayList;
import java.util.List;

import static virtual_robot.game_elements.classes.Ring.RingStatus.OFF_FIELD;

/**
 * For internal use only. Represents a robot with four mecanum wheels, a "scoop" at the front,
 * a rear intake, a shooter, a color sensor, four distance sensors, and BNO055 imu.
 *
 * This configuration controlled by the dyn4j physics/collision engine, and configured to
 * interact with game elements in the UltimateGoal game.
 *
 * BetaBot is the controller class for the "ulti_bot.fxml" markup file.
 */
//@BotConfig(name = "Ulti Bot", filename = "ulti_bot")
public class UltiBot extends MecanumPhysicsBase implements ControlsElements {

    //Fixture to hold the collision geometry of the ring intake
    private BodyFixture intakeFixture = null;

    private List<Ring> loadedRings = new ArrayList<>();     //List of loaded rings
    private Ring ringToLoad = null;                         //Ring pending loading, if any
    @FXML private Group loadedRingGroup;      // Group to represent loaded rings in the UI

    // States of the system that pushes loaded rings into the shooter mechanism
    enum KickerState {COCKING, COCKED, SHOOTING, SHOOT, SHOT_DONE}

    private ServoImpl kickerServo;
    ElapsedTime kickerTimer = new ElapsedTime();
    private KickerState kickerState = KickerState.COCKED;

    // Motors for intake, shooting, and scoop
    private DcMotorExImpl intakeMotor;
    private DcMotorExImpl shooterMotor;
    private DcMotorExImpl scoopMotor;

    //Scoop mechanism
    Body scoopBody;           // Body to represent the scoop in the physics engine
    Slide scoopSlide;           // The Slide joint that will connect scoop to the bot chassis
    @FXML Group scoopGroup;     // The Group to represent the scoop in the JavaFX UI
    Translate scoopTranslate = new Translate(0, 0);   // Transform to control position of scoopGroup
    double scoopPosition = 0;   // Current position of scoop in pixel units

    public UltiBot() {
        super();
    }

    public void initialize() {
        super.initialize();
        hardwareMap.setActive(true);
        kickerServo = hardwareMap.get(ServoImpl.class, "kicker_servo");
        intakeMotor = hardwareMap.get(DcMotorExImpl.class, "intake_motor");
        shooterMotor = hardwareMap.get(DcMotorExImpl.class, "shooter_motor");
        scoopMotor = hardwareMap.get(DcMotorExImpl.class, "scoop_motor");
        hardwareMap.setActive(false);

        /*
         * Add a BodyFixture at the back of the box, with a rectangular shape, to represent the intake.
         * This is a "sensor" fixture, meaning that it detects collisions, but does not "resolve" them.
         */
        intakeFixture = chassisBody.addFixture(new Rectangle(
                14 / VirtualField.INCHES_PER_METER, 4 / VirtualField.INCHES_PER_METER));
        intakeFixture.getShape().translate(0, -7 / VirtualField.INCHES_PER_METER);
        intakeFixture.setSensor(true);

        /*
         * Create a Body to represent a "scoop" (really just a pair of retractible prongs) to be used in
         * moving the wobbles arount. This body will have two fixtures, each of which has a thin rectangular
         * shape, one positioned on the right and the other on the left.
         *
         * The scoop body will join to the chassis body by a Slide joint. This joint has a motor that
         * allows the scoop position relative to the chassis to be controlled.
         */

//        The conventional way of creating the scoop Body:
//
//        scoopBody = new Body();
//
//        /*
//         * make the long skinny fixtures a little larger (37x10 pixel units) than the graphical representation
//         * of the prongs (30x5), in order to account for "penetration slop".
//         */
//        BodyFixture leftScoopFixture = scoopBody.addFixture(
//                new Rectangle(10/VirtualField.PIXELS_PER_METER, 37/VirtualField.PIXELS_PER_METER),
//                1, 0, 0);
//        BodyFixture rightScoopFixture = scoopBody.addFixture(
//                new Rectangle(10/VirtualField.PIXELS_PER_METER, 37/VirtualField.PIXELS_PER_METER),
//                1, 0, 0);
//        //Position the scoop fixture shapes to the left and right of midline
//        leftScoopFixture.getShape().translate(-23/VirtualField.PIXELS_PER_METER, 0);
//        rightScoopFixture.getShape().translate(23/VirtualField.PIXELS_PER_METER, 0);
//        scoopBody.setMass(MassType.NORMAL);
//        world.addBody(scoopBody);
//        leftScoopFixture.setFilter(Filters.CHASSIS_FILTER);
//        rightScoopFixture.setFilter(Filters.CHASSIS_FILTER);
//        // Position the scoop body over front part of chassis
//        scoopBody.translate(0, 22.5 / VirtualField.PIXELS_PER_METER);

        /*
         * Use Dyn4jUtil.createBody to generate dyn4j body from the JavaFX Group for the scoop.
         */
        FixtureData scoopBodyData = new FixtureData(Filters.CHASSIS_FILTER, 1.0, 0.0, 0.0,
                false, 2.0, 37.0/30.0);
        scoopBody = Dyn4jUtil.createBody(scoopGroup, this, 9, 9, scoopBodyData);
        world.addBody(scoopBody);

        // Because the prongs are skinny, set as bullet (not sure this helps at all)
        scoopBody.setBullet(true);

        // Create a Slide (specialized motorized Prismatic Joint) and add it to the world
        scoopSlide = new Slide(chassisBody, scoopBody, new Vector2(0,0), new Vector2(0,-1),
                VirtualField.Unit.PIXEL);
        world.addJoint(scoopSlide);
        scoopGroup.getTransforms().add(scoopTranslate);

        /*
         * Add a collision listener to the dyn4j world. This will handle collisions where the bot needs
         * to take control of a game element.
         */
        world.addCollisionListener(new CollisionListenerAdapter<Body, BodyFixture>(){
            @Override
            public boolean collision(NarrowphaseCollisionData<Body, BodyFixture> collision) {
                return handleNarrowPhaseCollisions(collision);
            }
        });

        loadedRingGroup.setVisible(false);

    }

    protected void createHardwareMap(){
        super.createHardwareMap();
        hardwareMap.put("kicker_servo", new ServoImpl());
        hardwareMap.put("intake_motor", new DcMotorExImpl(MotorType.Neverest40));
        hardwareMap.put("shooter_motor", new DcMotorExImpl(MotorType.Neverest40));
        hardwareMap.put("scoop_motor", new DcMotorExImpl(MotorType.Neverest40));
    }

    public synchronized void updateStateAndSensors(double millis){
        super.updateStateAndSensors(millis);

        // The intake and shooter motors must be updated each cycle, even if not using the result
        intakeMotor.update(millis);
        shooterMotor.update(millis);

        // If a Ring has been assigned to ringToLoad (in a Listener), load the ring
        if (ringToLoad != null) loadRing();

        /*
         * Handle shooting with the kicker servo
         */
        switch (kickerState){
            case COCKED:
                if (kickerServo.getInternalPosition() > 0.2) {
                    kickerState = KickerState.SHOOTING;
                    kickerTimer.reset();
                }
                break;
            case SHOOTING:
                if (kickerServo.getInternalPosition() <= 0.2) {
                    kickerState = KickerState.COCKING;
                    kickerTimer.reset();
                } else if (kickerServo.getInternalPosition() > 0.8 && kickerTimer.milliseconds() > 250){
                    kickerState = KickerState.SHOOT;
                    kickerTimer.reset();
                }
                break;
            case SHOOT:
                shootRing();
                kickerState = KickerState.SHOT_DONE;
                break;
            case SHOT_DONE:
                if (kickerServo.getInternalPosition() <= 0.8) {
                    kickerState = KickerState.COCKING;
                    kickerTimer.reset();
                }
                break;
            case COCKING:
                if (kickerServo.getInternalPosition() > 0.8) {
                    kickerState = KickerState.SHOT_DONE;
                    kickerTimer.reset();
                } else if (kickerServo.getInternalPosition() <= 0.2 && kickerTimer.milliseconds() > 250){
                    kickerState = KickerState.COCKED;
                }
                break;
        }

        /*
         * Handle positioning of the scoop, using the interval ticks of the scoop motor.
         */
        double scoopTicks = scoopMotor.update(millis);
        scoopPosition += scoopTicks * 30.0 / 1800.0;    //Scoop position in pixels
        if (scoopPosition > 25) scoopPosition = 25;
        else if (scoopPosition < 0) scoopPosition = 0;
        scoopSlide.setPosition(scoopPosition);
    }

    public synchronized void updateDisplay(){
        super.updateDisplay();
        // If there are any loaded rings, display a ring over center of bot
        loadedRingGroup.setVisible(!loadedRings.isEmpty());
        // Display the scoop in the appropriate position
        scoopTranslate.setY(-scoopPosition);
    }

    public void powerDownAndReset(){
        super.powerDownAndReset();
    }

    /**
     * Listener method to handle Narrowphase collision. This method will look specifically for collisions that cause
     * the robot to control a previously un-controlled game element. This method is called DURING the world
     * update.
     *
     * Note that when ring collision with the intake is detected, the ring body is not immediately removed from
     * the dyn4j world. During so DURING the world update is not recommended (per dyn4j docs), and does cause
     * problems. Instead, save a reference to the ring to be loaded, and handle AFTER the world update, within
     * the updateStateAndSensors method.
     *
     * @param collision
     * @return True to allow collision resolution to continue; False to terminate collision resolution.
     */
    private boolean handleNarrowPhaseCollisions(NarrowphaseCollisionData<Body, BodyFixture> collision){
        BodyFixture f1 = collision.getFixture1();
        BodyFixture f2 = collision.getFixture2();
        if ((f1 == intakeFixture || f2 == intakeFixture) &&  loadedRings.size() < 3
                && ringToLoad == null) {
            Body b = f1 == intakeFixture ? collision.getBody2() : collision.getBody1();
            double intakeVel = intakeMotor.getVelocity();
            boolean intakeFwd = intakeMotor.getDirection() == DcMotorSimple.Direction.FORWARD;
            boolean intakeOn = intakeVel>560 && intakeFwd || intakeVel<-560 && !intakeFwd;
            if (b.getUserData() instanceof Ring && intakeOn){
                Ring r = (Ring)b.getUserData();
                if (!(r.getStatus() == Ring.RingStatus.FLYING)) {
                    ringToLoad = r;
                    return false;
                }
            }
        }
        return true;
    }

    private void loadRing(){
        ringToLoad.setStatus(Ring.RingStatus.CONTROLLED);
        if (!loadedRings.contains(ringToLoad)) loadedRings.add(ringToLoad);
        ringToLoad = null;
    }

    private void shootRing(){
        double shooterVel = shooterMotor.getVelocity();
        boolean shooterFwd = shooterMotor.getDirection() == DcMotorSimple.Direction.FORWARD;
        boolean shooterOn = shooterVel > 200 && shooterFwd || shooterVel < -200 && !shooterFwd;
        if (loadedRings.isEmpty() || !shooterOn) return;
        Ring r = loadedRings.remove(0);
        r.setStatus(Ring.RingStatus.FLYING);
        // Initial position and velocity of the shot ring (meters and meters/sec)
        Vector2 pos = chassisBody.getWorldPoint(new Vector2(0, 12 / VirtualField.INCHES_PER_METER));
        Vector2 vel = chassisBody.getWorldVector(new Vector2(0, 60 / VirtualField.INCHES_PER_METER));
        vel.add(new Vector2(0, 12 / VirtualField.INCHES_PER_METER).cross(chassisBody.getAngularVelocity()));
        vel.multiply(Math.abs(shooterVel) / 1120);
        r.setLocationMeters(pos);
        r.setVelocityMetersPerSec(vel.x, vel.y);
    }

    /**
     * Preload rings (if Off-Field Rings are available)
     * @param game
     */
    @Override
    public void preloadElements(Game game){
        if (!(game instanceof UltimateGoal)) return;
        if (ringToLoad != null) {
            ringToLoad.setStatus(OFF_FIELD);
            ringToLoad = null;
        }
        while (loadedRings.size() < 3 && Ring.ringsOffField.size() > 0){
            Ring r = Ring.ringsOffField.remove(0);
            loadedRings.add(r);
            r.setStatus(Ring.RingStatus.CONTROLLED);
        }
        updateDisplay();
    }

    /**
     * Remove any loaded rings and set them to OFF_FIELD status.
     * @param game
     */
    @Override
    public void clearLoadedElements(Game game){
        if (!(game instanceof UltimateGoal)) return;
        if (ringToLoad != null) {
            ringToLoad.setStatus(OFF_FIELD);
            ringToLoad = null;
        }
        while (loadedRings.size() > 0){
            Ring r = loadedRings.get(0);
            r.setStatus(OFF_FIELD);
            loadedRings.remove(r);
        }
        updateDisplay();
    }

}
