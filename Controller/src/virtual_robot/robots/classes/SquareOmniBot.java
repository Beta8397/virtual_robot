package virtual_robot.robots.classes;

import com.qualcomm.robotcore.hardware.DeadWheelEncoder;
import com.qualcomm.robotcore.hardware.ServoImpl;
import com.qualcomm.robotcore.hardware.configuration.MotorType;
import javafx.fxml.FXML;
import javafx.scene.shape.Rectangle;
import javafx.scene.transform.Rotate;
import virtual_robot.controller.BotConfig;
import virtual_robot.util.AngleUtils;

/**
 * For internal use only. Represents a robot with four mecanum wheels, color sensor, four distance sensors,
 * a BNO055IMU, a Servo-controlled arm on the back, and three dead-wheel encoder pods
 *
 * MecanumBot is the controller class for the "mecanum_bot.fxml" markup file.
 *
 */
//@BotConfig(name = "Square Omni Bot", filename = "square_omni_bot")
public class SquareOmniBot extends SquareOmniPhysicsBase {

    public SquareOmniBot(){
        super();
    }

    public void initialize(){
        super.initialize();
    }


}
