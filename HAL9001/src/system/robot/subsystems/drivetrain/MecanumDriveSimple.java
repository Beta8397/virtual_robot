package system.robot.subsystems.drivetrain;

import com.qualcomm.robotcore.hardware.DcMotor;
import system.config.AutonomousConfig;
import system.config.ConfigData;
import system.config.ConfigParam;
import system.config.TeleopConfig;
import system.robot.Robot;
import system.robot.localizer.HolonomicDriveEncoderLocalizer;
import util.control.Button;
import util.control.CustomizableGamepad;
import util.control.Toggle;
import util.math.FakeNumpy;
import util.math.geometry.Vector2D;

import static java.lang.Math.*;

public class MecanumDriveSimple extends HolonomicDrivetrain {

    protected static final String
            DRIVE_STICK = "Drive Stick",
            TURN_STICK = "Turn Stick",
            SPEED_TOGGLE = "Speed Toggle",
            TURN_SPEED_TOGGLE = "Turn Speed Toggle",
            TURN_LEFT_BUTTON = "Turn Left Button",
            TURN_RIGHT_BUTTON = "Turn Right Button";

    private final Toggle
            speedToggle = new Toggle(Toggle.ToggleTypes.flipToggle, false),
            turnSpeedToggle = new Toggle(Toggle.ToggleTypes.flipToggle, false);

    private CustomizableGamepad gamepad;
    protected final String TOP_LEFT, TOP_RIGHT, BOT_LEFT, BOT_RIGHT;

    private double
            speedToggleMultiplier = 0.5,
            turnSpeedToggleMultiplier = 0.5;
    private double turnButtonPower = 0.3;

    public enum ReverseType {
        LEFT,
        RIGHT,
        FRONT,
        BACK
    }

    public MecanumDriveSimple(Robot robot, DriveConfig driveConfig, String topLeft, String topRight, String botLeft, String botRight, boolean useConfig) {
        super(robot, driveConfig, topLeft, topRight, botLeft, botRight);

        this.localizer = new HolonomicDriveEncoderLocalizer(
                this,
                topLeft,
                topRight,
                botLeft,
                botRight
        );

        usesConfig = useConfig;

        TOP_LEFT = topLeft;
        TOP_RIGHT = topRight;
        BOT_LEFT = botLeft;
        BOT_RIGHT = botRight;

        setReverseType(ReverseType.LEFT);

        gamepad = new CustomizableGamepad(robot);
        gamepad.addButton(DRIVE_STICK, new Button<Vector2D>(1, Button.VectorInputs.right_stick));
        gamepad.addButton(TURN_STICK, new Button<Double>(1, Button.DoubleInputs.left_stick_x));
        gamepad.addButton(SPEED_TOGGLE, new Button<Boolean>(1, Button.BooleanInputs.noButton));
        gamepad.addButton(TURN_SPEED_TOGGLE, new Button<Boolean>(1, Button.BooleanInputs.noButton));
        gamepad.addButton(TURN_LEFT_BUTTON, new Button<Boolean>(1, Button.BooleanInputs.noButton));
        gamepad.addButton(TURN_RIGHT_BUTTON, new Button<Boolean>(1, Button.BooleanInputs.noButton));
    }

    public MecanumDriveSimple(Robot robot, DriveConfig driveConfig, String topLeft, String topRight, String botLeft, String botRight) {
        this(robot, driveConfig, topLeft, topRight, botLeft, botRight, true);
    }

    public void setReverseType(ReverseType reverseType) {
        switch (reverseType) {
            case LEFT:
                setMotorDirection(TOP_LEFT, DcMotor.Direction.REVERSE);
                setMotorDirection(BOT_LEFT, DcMotor.Direction.REVERSE);
                setMotorDirection(TOP_RIGHT, DcMotor.Direction.FORWARD);
                setMotorDirection(BOT_RIGHT, DcMotor.Direction.FORWARD);
                break;
            case RIGHT:
                setMotorDirection(TOP_LEFT, DcMotor.Direction.FORWARD);
                setMotorDirection(BOT_LEFT, DcMotor.Direction.FORWARD);
                setMotorDirection(TOP_RIGHT, DcMotor.Direction.REVERSE);
                setMotorDirection(BOT_RIGHT, DcMotor.Direction.REVERSE);
                break;
            case FRONT:
                setMotorDirection(TOP_LEFT, DcMotor.Direction.REVERSE);
                setMotorDirection(BOT_LEFT, DcMotor.Direction.FORWARD);
                setMotorDirection(TOP_RIGHT, DcMotor.Direction.REVERSE);
                setMotorDirection(BOT_RIGHT, DcMotor.Direction.FORWARD);
                break;
            case BACK:
                setMotorDirection(TOP_LEFT, DcMotor.Direction.FORWARD);
                setMotorDirection(BOT_LEFT, DcMotor.Direction.REVERSE);
                setMotorDirection(TOP_RIGHT, DcMotor.Direction.FORWARD);
                setMotorDirection(BOT_RIGHT, DcMotor.Direction.REVERSE);
                break;
        }
    }

    public void setToggleableVelocityMultiplier(double velocityMultiplier) {
        speedToggleMultiplier = abs(velocityMultiplier);
    }

    public void setToggleableTurnSpeedMultiplier(double turnSpeedMultiplier) {
        turnSpeedToggleMultiplier = abs(turnSpeedMultiplier);
    }

    public void setTurnButtonPower(double turnButtonPower) {
        this.turnButtonPower = turnButtonPower;
    }

    public void setDriveStick(Button<Vector2D> driveStick) {
        gamepad.addButton(DRIVE_STICK, driveStick);
    }

    public void setTurnStick(Button<Double> turnStick) {
        gamepad.addButton(TURN_STICK, turnStick);
    }

    public void setSpeedToggleButton(Button<Boolean> speedToggleButton) {
        gamepad.addButton(SPEED_TOGGLE, speedToggleButton);
    }

    public void setTurnSpeedToggleButton(Button<Boolean> turnSpeedToggleButton) {
        gamepad.addButton(TURN_SPEED_TOGGLE, turnSpeedToggleButton);
    }

    public void setTurnLeftButton(Button<Boolean> turnLeftButton) {
        gamepad.addButton(TURN_LEFT_BUTTON, turnLeftButton);
    }

    public void setTurnRightButton(Button<Boolean> turnRightButton) {
        gamepad.addButton(TURN_RIGHT_BUTTON, turnRightButton);
    }

    public void setPower(double topLeftPower, double topRightPower, double botLeftPower, double botRightPower) {
        double[] powers = new double[] {topLeftPower, topRightPower, botLeftPower, botRightPower};

        double max = FakeNumpy.max(FakeNumpy.abs(powers));
        FakeNumpy.divide(powers, max > 1 ? max : 1);

        setMotorPower(TOP_LEFT, powers[0]);
        setMotorPower(TOP_RIGHT, powers[1]);
        setMotorPower(BOT_LEFT, powers[2]);
        setMotorPower(BOT_RIGHT, powers[3]);
    }

    @Override
    public void turnPowerInternal(double power) {
        setPower(-power, power, -power, power);
    }

    @Override
    protected void movePowerInternal(Vector2D power) {
        Vector2D transformedPowerVector = power.clone().rotate(PI/4);
        setPower(
                transformedPowerVector.getX(),
                transformedPowerVector.getY(),
                transformedPowerVector.getY(),
                transformedPowerVector.getX()
        );
    }

    public void arcPower(Vector2D velocity, double turnPower) {
        Vector2D transformedPowerVector = modifyPower(velocity).rotate(PI/4);
        turnPower = modifyTurnPower(turnPower);

        setPower(
                transformedPowerVector.getX()-turnPower,
                transformedPowerVector.getY()+turnPower,
                transformedPowerVector.getY()-turnPower,
                transformedPowerVector.getX()+turnPower
        );
    }

    @Override
    public void init() {

    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {
        if(usesConfig) {
            gamepad = robot.pullControls(this);
            ConfigData configData = robot.pullNonGamepad(this);

            driveMode = configData.getData("Drive Mode", DriveMode.class);

            velocityScaleMethod = configData.getData("Velocity Scaling Method", SpeedScaleMethod.class);
            turnSpeedScaleMethod = configData.getData("Turn Speed Scaling Method", SpeedScaleMethod.class);

            constantSpeedMultiplier = configData.getData("Velocity Multiplier", Double.class);
            speedToggleMultiplier = configData.getData("Speed Toggle Multiplier", Double.class);
            velocityCap = configData.getData("Velocity Cap", Double.class);

            constantTurnSpeedMultiplier = configData.getData("Turn Speed Multiplier", Double.class);
            turnSpeedToggleMultiplier = configData.getData("Turn Speed Toggle Multiplier", Double.class);
            turnSpeedCap = configData.getData("Turn Speed Cap", Double.class);

            turnButtonPower = configData.getData("Turn Button Power", Double.class);
        }
    }

    @Override
    public void handle() {
        localizer.update();

        if(driveMode != DriveMode.DISABLED) {
            speedToggle.updateToggle(gamepad.getInput(SPEED_TOGGLE));
            turnSpeedToggle.updateToggle(gamepad.getInput(TURN_SPEED_TOGGLE));

            currentSpeedMultiplier = speedToggle.getCurrentState() ? speedToggleMultiplier : 1;
            currentTurnSpeedMultiplier = turnSpeedToggle.getCurrentState() ? turnSpeedToggleMultiplier : 1;

            Vector2D inputVelocity = gamepad.getInput(DRIVE_STICK);
            inputVelocity.rotate(
                    driveMode == DriveMode.STANDARD ? -PI/2 : -(PI/2 + localizer.getPoseEstimate().getHeading())
            );

            double turnPower = gamepad.getInput(TURN_LEFT_BUTTON) ? turnButtonPower : gamepad.getInput(TURN_RIGHT_BUTTON) ? -turnButtonPower : 0;
            turnPower = turnPower == 0 ? gamepad.getInput(TURN_STICK) : turnPower;
            turnPower = -turnPower;

            if(turnPower == 0) {
                double correction = -headingController.update(localizer.getPoseEstimate().getHeading(), localizer.getPoseVelocity().getHeading());
                turnPower += abs(headingController.getLastError()) < headingAngleToleranceRadians ? 0 : correction;
            }
            else {
                headingController.setTargetPosition(localizer.getPoseEstimate().getHeading());
            }

            arcPower(inputVelocity, turnPower);
        }
    }

    @Override
    public void stop() {
        stopAllMotors();
    }

    @AutonomousConfig
    public static ConfigParam[] autonomousConfig() {
        return new ConfigParam[] {
                new ConfigParam("Drive Mode", DriveMode.STANDARD),
                new ConfigParam("Velocity Scaling Method", SpeedScaleMethod.NONE),
                new ConfigParam("Turn Speed Scaling Method", SpeedScaleMethod.NONE),
                new ConfigParam("Velocity Multiplier", ConfigParam.numberMap(0, 10, 0.05), 1.0),
                new ConfigParam("Speed Toggle Multiplier", ConfigParam.numberMap(0, 10, 0.05), 0.5),
                new ConfigParam("Velocity Cap", ConfigParam.numberMap(0,1,0.05),1.0),
                new ConfigParam("Turn Speed Multiplier", ConfigParam.numberMap(0, 10, 0.05), 1.0),
                new ConfigParam("Turn Speed Toggle Multiplier", ConfigParam.numberMap(0, 10, 0.05), 0.5),
                new ConfigParam("Turn Speed Cap", ConfigParam.numberMap(0,1,0.05),1.0),
                new ConfigParam("Turn Button Power", ConfigParam.numberMap(0, 1, 0.05), 0.3)
        };
    }

    @TeleopConfig
    public static ConfigParam[] teleopConfig() {
        return new ConfigParam[] {
                new ConfigParam("Drive Mode", DriveMode.STANDARD),
                new ConfigParam(DRIVE_STICK, Button.VectorInputs.right_stick),
                new ConfigParam(TURN_STICK, Button.DoubleInputs.left_stick_x),
                new ConfigParam(TURN_LEFT_BUTTON, Button.BooleanInputs.noButton),
                new ConfigParam(TURN_RIGHT_BUTTON, Button.BooleanInputs.noButton),
                new ConfigParam(SPEED_TOGGLE, Button.BooleanInputs.noButton),
                new ConfigParam(TURN_SPEED_TOGGLE, Button.BooleanInputs.noButton),
                new ConfigParam("Velocity Scaling Method", SpeedScaleMethod.NONE),
                new ConfigParam("Turn Speed Scaling Method", SpeedScaleMethod.NONE),
                new ConfigParam("Velocity Multiplier", ConfigParam.numberMap(0, 10, 0.05), 1.0),
                new ConfigParam("Speed Toggle Multiplier", ConfigParam.numberMap(0, 10, 0.05), 0.5),
                new ConfigParam("Velocity Cap", ConfigParam.numberMap(0,1,0.05),1.0),
                new ConfigParam("Turn Speed Multiplier", ConfigParam.numberMap(0, 10, 0.05), 1.0),
                new ConfigParam("Turn Speed Toggle Multiplier", ConfigParam.numberMap(0, 10, 0.05), 0.5),
                new ConfigParam("Turn Speed Cap", ConfigParam.numberMap(0,1,0.05),1.0),
                new ConfigParam("Turn Button Power", ConfigParam.numberMap(0, 1, 0.05), 0.3)
        };
    }
}