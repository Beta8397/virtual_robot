package virtual_robot.controller.robots.classes;

import com.qualcomm.robotcore.hardware.ServoImpl;
import javafx.fxml.FXML;
import javafx.scene.Group;
import javafx.scene.shape.Rectangle;
import javafx.scene.transform.Rotate;
import org.dyn4j.dynamics.Body;
import org.dyn4j.dynamics.BodyFixture;
import org.dyn4j.geometry.Convex;
import org.dyn4j.geometry.Vector2;
import virtual_robot.controller.BotConfig;
import virtual_robot.dyn4j.Dyn4jUtil;
import virtual_robot.dyn4j.FixtureData;

/**
 * For internal use only. Represents a robot with four mechanum wheels, color sensor, four distance sensors,
 * a BNO055IMU, and a Servo-controlled arm on the back.
 * <p>
 * MechanumBot is the controller class for the "mechanum_bot.fxml" markup file.
 */
@BotConfig(name = "Test Bot", filename = "test_bot")
public class TestBot extends MechanumBase {

    // backServoArm is instantiated during loading via a fx:id property.
    ServoImpl servo;
    @FXML Rectangle rect1;
    @FXML Rectangle rect0;
    @FXML
    Group group0;

    public TestBot() {
        super();
    }

    public void initialize() {
        super.initialize();
        hardwareMap.setActive(true);
        servo = (ServoImpl) hardwareMap.servo.get("back_servo");
        hardwareMap.setActive(false);

        FixtureData fd = null;
        Body body = Dyn4jUtil.createBody(group0, fd);

        System.out.println("\nBody position: " + body.getTransform().getTranslationX()/0.0254
        + "  " + body.getTransform().getTranslationY()/0.0254);
        for (BodyFixture bf: body.getFixtures()){
            System.out.println("FIXTURE");
            org.dyn4j.geometry.Rectangle r = (org.dyn4j.geometry.Rectangle)bf.getShape();
            for (Vector2 v: r.getVertices()){
                System.out.println("" + v.x/0.0254 + "  " + v.y/0.0254);
            }
        }
    }

    protected void createHardwareMap(){
        super.createHardwareMap();
        hardwareMap.put("back_servo", new ServoImpl());
    }

    public synchronized void updateStateAndSensors(double millis){
        super.updateStateAndSensors(millis);
    }

    public synchronized void updateDisplay(){
        super.updateDisplay();
    }

    public void powerDownAndReset(){
        super.powerDownAndReset();
    }
}
