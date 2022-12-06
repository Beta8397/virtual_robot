package virtual_robot.robots.classes;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.BNO055IMUImpl;
import com.qualcomm.hardware.bosch.BNO055IMUNew;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.hardware.configuration.MotorType;
import javafx.application.Platform;
import javafx.beans.value.ChangeListener;
import javafx.beans.value.ObservableValue;
import javafx.fxml.FXML;
import javafx.geometry.Bounds;
import javafx.scene.Group;
import javafx.scene.control.Button;
import javafx.scene.control.Slider;
import javafx.scene.input.MouseDragEvent;
import javafx.scene.input.MouseEvent;
import javafx.scene.paint.Color;
import javafx.scene.shape.Circle;
import javafx.scene.shape.Rectangle;
import javafx.scene.transform.Rotate;
import javafx.scene.transform.Scale;
import javafx.scene.transform.Translate;
import virtual_robot.controller.BotConfig;
import virtual_robot.controller.VirtualBot;
import virtual_robot.controller.VirtualField;
import virtual_robot.util.AngleUtils;

/**
 * For internal use only. Represents a robot with two standard wheels, color sensor, four distance sensors,
 * a Gyro Sensor, and a Servo-controlled arm on the back.
 *
 * TwoWheelBot is the controller class for the "two_wheel_bot.fxml" markup file.
 */
@BotConfig(name = "Programming Board", filename = "programming_board")
public class ProgrammingBoard extends VirtualBot {

    private MotorType MOTOR_TYPE = MotorType.Neverest40;
    private DcMotorExImpl motor = null;
    private BNO055IMUImpl imu = null;
    private BNO055IMUNew imuNew = null;
    private PassiveColorSensorImpl colorSensor = null;
    private ServoImpl servo = null;
    private DigitalChannelImpl digitalChannel = null;
    private AnalogInput analogInput = null;
    private PassiveDistanceSensorImpl distanceSensor = null;

    @FXML private Group propGroup;
    @FXML private Group servoArmGroup;
    @FXML private Button btnTouch;
    @FXML private Slider sldPot;
    @FXML private Slider sldRed;
    @FXML private Slider sldGreen;
    @FXML private Slider sldBlue;
    @FXML private Circle circleColor;
    @FXML private Rectangle rectBoard;
    @FXML private Slider sldDist;

    private Rotate servoArmGroupRotate = new Rotate(0, 25, 25);
    private Rotate propGroupRotate = new Rotate(0, 50, 50);
    double propAngleDegrees = 0;

    private double voltage = 0;
    private volatile int red = 0;
    private volatile int green = 0;
    private volatile int blue = 0;
    private volatile boolean btnTouchPushed = false;

    private double initMouseX = 0;
    private double initMouseY = 0;
    private double initMouseAngle = 0;
    private double initHeading = 0;
    private double fieldCenterX = 0;
    private double fieldCenterY = 0;

    private double boardWidth = 0;


    private ChangeListener<Number> sliderColorChangeListener = new ChangeListener<Number>() {
        @Override
        public void changed(ObservableValue<? extends Number> observable, Number oldValue, Number newValue) {
            red = (int)sldRed.getValue();
            green = (int)sldGreen.getValue();
            blue = (int)sldBlue.getValue();
            Platform.runLater(new Runnable() {
                @Override
                public void run() {
                    circleColor.setFill(Color.color((double)red/255.0, (double)green/255.0, (double)blue/255.0));
                }
            });
        }
    };


    public ProgrammingBoard(){
        super();
    }

    public void initialize(){
        super.initialize();

        hardwareMap.setActive(true);
        motor = (DcMotorExImpl)hardwareMap.get(DcMotorEx.class, "motor");
        imu = (BNO055IMUImpl) hardwareMap.get(BNO055IMU.class, "imu");
        imuNew = hardwareMap.get(BNO055IMUNew.class, "imu");
        colorSensor = (PassiveColorSensorImpl)hardwareMap.colorSensor.get("sensor_color_distance");
        servo = (ServoImpl)hardwareMap.servo.get("servo");
        digitalChannel = (DigitalChannelImpl) hardwareMap.get(DigitalChannel.class, "touch_sensor");
        analogInput = hardwareMap.get(AnalogInput.class, "pot");
        distanceSensor = (PassiveDistanceSensorImpl)hardwareMap.get(PassiveDistanceSensorImpl.class, "sensor_color_distance");
        hardwareMap.setActive(false);

        propGroup.getTransforms().add(propGroupRotate);
        servoArmGroup.getTransforms().add(servoArmGroupRotate);
        Bounds boundsInScene = fieldPane.localToScene(fieldPane.getBoundsInLocal());
        fieldCenterX = boundsInScene.getMinX() + VirtualField.HALF_FIELD_WIDTH;
        fieldCenterY = boundsInScene.getMinY() + VirtualField.HALF_FIELD_WIDTH;
        sldPot.setMax(analogInput.getMaxVoltage());
        sldRed.valueProperty().addListener(sliderColorChangeListener);
        sldGreen.valueProperty().addListener(sliderColorChangeListener);
        sldBlue.valueProperty().addListener(sliderColorChangeListener);

        boardWidth = rectBoard.getWidth() * VirtualField.FIELD_WIDTH / 600;
    }

    protected void createHardwareMap(){
        hardwareMap = new HardwareMap();
        hardwareMap.put("motor", new DcMotorExImpl(MOTOR_TYPE));
        hardwareMap.put("imu", new BNO055IMUImpl(this, 10));
        hardwareMap.put("imu", new BNO055IMUNew(this, 10));
        hardwareMap.put("sensor_color_distance", new PassiveColorSensorImpl());
        hardwareMap.put("servo", new ServoImpl());
        hardwareMap.put("touch_sensor", new DigitalChannelImpl());
        hardwareMap.put("pot", new AnalogInput(5.0));
        hardwareMap.put("sensor_color_distance", new PassiveDistanceSensorImpl(50, 250,
                50, 250));
    }

    public synchronized void updateStateAndSensors(double millis){
        propAngleDegrees = AngleUtils.normalizeDegrees(propAngleDegrees +
                motor.update(millis) * 360.0 / motor.MOTOR_TYPE.TICKS_PER_ROTATION);
        analogInput.update(sldPot.getValue());
        imu.updateHeadingRadians(headingRadians);
        imuNew.updateHeadingRadians(headingRadians);
        colorSensor.updateColor(red, green, blue);
        digitalChannel.update(!btnTouchPushed);
        distanceSensor.update(sldDist.getValue());
    }

    public synchronized void updateDisplay(){
        super.updateDisplay();
        servoArmGroupRotate.setAngle(-180.0 * servo.getInternalPosition());
        propGroupRotate.setAngle(propAngleDegrees);
    }

    public synchronized void powerDownAndReset(){
        headingRadians = 0;
        motor.stopAndReset();
        imu.close();
        if (Platform.isFxApplicationThread()) updateDisplay();
        else {
            Platform.runLater(new Runnable() {
                @Override
                public void run() {
                    updateDisplay();
                }
            });
        }
    }

    /**
     * Because the programming board will be 500x500 pixels (rather than 75x75), it is necessary to override the
     * setUpDisplayGroup method.
     *
     */
    protected void setUpDisplayGroup(Group group){

        displayGroup = group;

        displayGroup.getTransforms().add(new Translate(VirtualField.HALF_FIELD_WIDTH - boardWidth/2,
                VirtualField.HALF_FIELD_WIDTH - boardWidth/2));
        botTranslate = new Translate(0,0);
        displayGroup.getTransforms().add(botTranslate);
        botRotate = new Rotate(0, boardWidth/2, boardWidth/2);
        displayGroup.getTransforms().add(botRotate);
        displayGroup.getTransforms().add(new Scale(VirtualField.FIELD_WIDTH/600, VirtualField.FIELD_WIDTH/600));
        fieldPane.getChildren().add(displayGroup);
    }

    @FXML
    private synchronized void handleMouseEvents(MouseEvent event){
        if (event.getSource() == btnTouch){
            if (event.getEventType() == MouseEvent.MOUSE_PRESSED){
                btnTouchPushed = true;
            } else if (event.getEventType() == MouseEvent.MOUSE_RELEASED || event.getEventType() == MouseEvent.MOUSE_EXITED){
                btnTouchPushed = false;
            }
        } else if (event.getSource() == rectBoard){
            if (event.getEventType() == MouseDragEvent.DRAG_DETECTED){
                initMouseX = event.getSceneX() - fieldCenterX;
                initMouseY = fieldCenterY - event.getSceneY();
                initMouseAngle = Math.atan2(initMouseY, initMouseX);
                initHeading = headingRadians;
                rectBoard.startFullDrag();
            } else if (event.getEventType() == MouseDragEvent.MOUSE_DRAG_OVER){
                double newMouseX = event.getSceneX() - fieldCenterX;
                double newMouseY = fieldCenterY - event.getSceneY();
                double newMouseAngle = Math.atan2(newMouseY, newMouseX);
                double deltaMouseAngle = AngleUtils.normalizeRadians(newMouseAngle - initMouseAngle);
                headingRadians = AngleUtils.normalizeRadians(initHeading + deltaMouseAngle);
                Platform.runLater(new Runnable() {
                    @Override
                    public void run() {
                        updateDisplay();
                    }
                });
            }
        }
        event.consume();
    }



    /**
     * ProgrammingBoard doesn't support positioning with mouse click; override the VirtualBot method
     * @param arg
     */
    @Override
    public synchronized void positionWithMouseClick(MouseEvent arg){}

}
