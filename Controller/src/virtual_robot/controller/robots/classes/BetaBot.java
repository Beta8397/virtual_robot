package virtual_robot.controller.robots.classes;

import com.qualcomm.robotcore.hardware.ServoImpl;
import com.qualcomm.robotcore.util.ElapsedTime;
import javafx.fxml.FXML;
import javafx.scene.transform.Rotate;
import org.dyn4j.dynamics.BodyFixture;
import org.dyn4j.geometry.Rectangle;
import org.dyn4j.geometry.Transform;
import org.dyn4j.geometry.Vector2;
import org.dyn4j.world.NarrowphaseCollisionData;
import org.dyn4j.world.listener.CollisionListenerAdapter;
import virtual_robot.controller.BotConfig;
import virtual_robot.controller.VRBody;
import virtual_robot.controller.VirtualField;
import virtual_robot.controller.game_elements.classes.Ring;

import java.util.ArrayList;
import java.util.List;

/**
 * For internal use only. Represents a robot with four mechanum wheels, color sensor, four distance sensors,
 * a BNO055IMU, and a Servo-controlled arm on the back.
 * <p>
 * MechanumBot is the controller class for the "mechanum_bot.fxml" markup file.
 */
@BotConfig(name = "Beta Bot", filename = "beta_bot")
public class BetaBot extends MechanumBase {

    private BodyFixture intakeFixture = null;

    private List<Ring> loadedRings = new ArrayList<>();
    private Ring ringToLoad = null;

    private boolean intakeOn = true;

    enum KickerState {COCKING, COCKED, SHOOTING, SHOOT, SHOT_DONE}

    private ServoImpl kickerServo;
    ElapsedTime kickerTimer = new ElapsedTime();
    private KickerState kickerState = KickerState.COCKED;

    public BetaBot() {
        super();
    }

    public void initialize() {
        super.initialize();
        hardwareMap.setActive(true);
        kickerServo = hardwareMap.get(ServoImpl.class, "kicker_servo");
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
         * Add a collision listener to the dyn4j world. This will handle collisions where the bot needs
         * to take control of a game element.
         */
        world.addCollisionListener(new CollisionListenerAdapter<VRBody, BodyFixture>(){
            @Override
            public boolean collision(NarrowphaseCollisionData<VRBody, BodyFixture> collision) {
                return handleNarrowPhaseCollisions(collision);
            }
        });

    }

    protected void createHardwareMap(){
        super.createHardwareMap();
        hardwareMap.put("kicker_servo", new ServoImpl());
    }

    public synchronized void updateStateAndSensors(double millis){
        super.updateStateAndSensors(millis);

        if (ringToLoad != null) loadRing();

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
    }

    public synchronized void updateDisplay(){
        super.updateDisplay();
    }

    public void powerDownAndReset(){
        super.powerDownAndReset();
    }

    private boolean handleNarrowPhaseCollisions(NarrowphaseCollisionData<VRBody, BodyFixture> collision){
        BodyFixture f1 = collision.getFixture1();
        BodyFixture f2 = collision.getFixture2();
        if ((f1 == intakeFixture || f2 == intakeFixture) && intakeOn && loadedRings.size() < 3
                && ringToLoad == null) {
            VRBody b = f1 == intakeFixture ? collision.getBody2() : collision.getBody1();
            if (b.getParent() instanceof Ring){
                Ring r = (Ring)b.getParent();
                if (!r.isInFlight()) {
                    ringToLoad = r;
                    return false;
                }
            }
        }
        return true;
    }

    private void loadRing(){
        ringToLoad.setOnField(false);
        ringToLoad.setControlled(true);
        ringToLoad.setRolling(false);
        if (!loadedRings.contains(ringToLoad)) loadedRings.add(ringToLoad);
        ringToLoad = null;
    }

    private void shootRing(){
        if (loadedRings.isEmpty()) return;
        Ring r = loadedRings.remove(0);
        r.setInFlight(true);
        r.setControlled(false);
        // Initial position and velocity of the shot ring (meters and meters/sec)
        Vector2 pos = chassisBody.getWorldPoint(new Vector2(0, 12 / VirtualField.INCHES_PER_METER));
        Vector2 vel = chassisBody.getWorldVector(new Vector2(0, 48 / VirtualField.INCHES_PER_METER));
        System.out.println("Chassis: " + chassisBody.getTransform().getTranslationX()
                + "  " + chassisBody.getTransform().getTranslationY());
        System.out.println("Pos: " + pos);
        System.out.println("Vel: " + vel);
        r.setLocationMeters(pos);
        r.setVelocityMetersPerSec(vel.x, vel.y);
        r.setOnField(true);
    }

}
