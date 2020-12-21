package system.robot.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import system.config.AutonomousConfig;
import system.config.ConfigData;
import system.config.ConfigParam;
import system.config.TeleopConfig;
import system.robot.Robot;
import system.robot.SubSystem;
import util.control.Button;
import util.control.CustomizableGamepad;
import util.control.PIDController;
import util.control.Toggle;
import util.exceptions.DumpsterFireException;
import util.exceptions.InvalidMoveCommandException;
import util.exceptions.NotBooleanInputException;
import util.exceptions.NotDoubleInputException;
import util.math.geometry.Vector2D;
import util.misc.BaseParam;

import java.util.ArrayList;
import java.util.function.Supplier;

/**
 * A customizable tankdrive subsystem.
 *
 * @author Dylan Zueck, Crow Force
 * @since 1.0.0
 * @version 1.0.0
 *
 * Creation Date: 7/17/19.
 */
@SuppressWarnings({"WeakerAccess","unused"})
public class QuadWheelDrive extends SubSystem {

    //Whether or not the config is being set up with specific values.
    private static boolean useSpecific = false;
    //The drivetrain's four motors
    private DcMotor botLeft, botRight, topLeft, topRight;
    //A boolean determining if the robot should be allowed to turn and move simultaneously
    private boolean turnAndMove;
    //A toggle object that detects if a boolean input changes twice (like a square pulse)
    private Toggle speedToggle = new Toggle(Toggle.ToggleTypes.flipToggle, false);
    //Modifiers for speed and joystick operations
    private double currentSpeedModeModifier, speedModeModifier, constantSpeedModifier;
    //Object that stores wanted buttons and is used to retrieve button inputs.
    private CustomizableGamepad inputs;
    //Button names CustomizableGamepad will use.
    private static final String SPEEDMODEBUTTON = "speedModeButton", DRIVESTICK = "driveStick", TURNSTICK = "turnStick";

    /**
     * This constructor creates the drive system without using config.
     *
     * @param robot The robot we will be using.
     * @param params The parameters for the drive system.
     */
    public QuadWheelDrive(Robot robot, Params params){
        super(robot);

        inputs = new CustomizableGamepad(robot);

        setMotorConfiguration(params.topLeftMotor,params.topRightMotor,params.botLeftMotor,params.botRightMotor);
        setSpeedModeModifier(params.speedModeModifier);
        setConstantSpeedModifier(params.constantSpeedModifier);
        setTurnAndMove(params.turnAndMove);
        setDriveStick(params.buttonsToSet[0]);
        setTurnStick(params.buttonsToSet[1]);
        setSpeedMode(params.buttonsToSet[2]);
    }

    /**
     * This constructor creates the drive system using config for values that are not numbers.
     *
     * @param robot The robot we will be using.
     * @param params The parameters for the drive system.
     */
    public QuadWheelDrive(Robot robot, NumberParams params){
        super(robot);

        setMotorConfiguration(params.topLeftMotor,params.topRightMotor,params.botLeftMotor,params.botRightMotor);

        setSpeedModeModifier(params.speedModeModifier);
        setConstantSpeedModifier(params.constantSpeedModifier);

        usesConfig = true;
    }

    /**
     * This constructor creates the drive system using config.
     *
     * @param robot The robot we will be using.
     * @param topLeftConfiguration The top left motor's configuration name.
     * @param topRightConfiguration The top right motor's configuration name.
     * @param botLeftConfiguration The bottom left motor's configuration name.
     * @param botRightConfiguration The bottom right motor's configuration name.
     */
    public QuadWheelDrive(Robot robot, String topLeftConfiguration, String topRightConfiguration, String botLeftConfiguration, String botRightConfiguration){
        super(robot);

        setMotorConfiguration(topLeftConfiguration,topRightConfiguration,botLeftConfiguration,botRightConfiguration);

        usesConfig = true;
    }

    @Override
    public void init()
    {
        normalDirection();
        resetEncoders();
    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {
        if(usesConfig && robot.isTeleop()) {
            setUsingConfigs();
        }
        else if(usesConfig && robot.isAutonomous()) {
            setUsingConfigsAutonomous();
        }
    }

    @Override
    public void handle() {
        if(!inputs.checkNoButton(SPEEDMODEBUTTON)){
            speedToggle.updateToggle(inputs.getInput(SPEEDMODEBUTTON));
            if(speedToggle.getCurrentState()){
                currentSpeedModeModifier = speedModeModifier;
            }
            else {
                currentSpeedModeModifier = 1;
            }
        }
        //drives forward and turns at the same time
        if (turnAndMove) {
            if ((double) inputs.getInput(DRIVESTICK)!= 0 && (double) inputs.getInput(TURNSTICK) != 0) {
                turnAndMove(new Vector2D(inputs.getInput(DRIVESTICK), inputs.getInput(TURNSTICK)));
            } else if ((double) inputs.getInput(DRIVESTICK) != 0) {
                drive(inputs.getInput(DRIVESTICK));
            } else if ((double) inputs.getInput(TURNSTICK)!= 0){
                turn(inputs.getInput(TURNSTICK));
            } else {
                stopMovement();
            }
        }
        //drives forward and turns but not at the same time
        else {
            if ((double) inputs.getInput(TURNSTICK) != 0) {
                turn(inputs.getInput(TURNSTICK));
            } else if ((double) inputs.getInput(DRIVESTICK) != 0) {
                drive(inputs.getInput(DRIVESTICK));
            } else {
                stopMovement();
            }
        }
    }

    @Override
    public void stop() {
        stopMovement();
    }

    /**
     * Reverses direction of the robot.
     */
    public void reverseDirection(){
        botLeft.setDirection(DcMotor.Direction.FORWARD);
        topLeft.setDirection(DcMotor.Direction.FORWARD);
        botRight.setDirection(DcMotor.Direction.REVERSE);
        topLeft.setDirection(DcMotor.Direction.REVERSE);
    }

    /**
     * Sets the direction of the robot(default direction).
     */
    public void normalDirection(){
        botLeft.setDirection(DcMotor.Direction.REVERSE);
        topLeft.setDirection(DcMotor.Direction.REVERSE);
        botRight.setDirection(DcMotor.Direction.FORWARD);
        topLeft.setDirection(DcMotor.Direction.FORWARD);
    }

    /**
     * Resents the encoders.
     */
    public void resetEncoders(){
        botLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        botRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        topLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        topRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        botLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        botRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        topLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        topRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Makes the robot drive.
     *
     * @param speed Speed to drive forward is positive and reverse is negative.
     */
    public void drive(double speed){
        botLeft.setPower((speed * constantSpeedModifier) * currentSpeedModeModifier);
        botRight.setPower((speed * constantSpeedModifier) * currentSpeedModeModifier);
        topLeft.setPower((speed * constantSpeedModifier) * currentSpeedModeModifier);
        topRight.setPower((speed * constantSpeedModifier) * currentSpeedModeModifier);
    }

    /**
     * Makes the robot turn. (Counterclockwise Positive)
     *
     * @param speed Speed to turn at(-1)-(1) (positive speed is turn right & negative speed is turn left).
     */
    public void turn(double speed){
        botLeft.setPower(-(speed * constantSpeedModifier) * currentSpeedModeModifier);
        botRight.setPower((speed * constantSpeedModifier) * currentSpeedModeModifier);
        topLeft.setPower(-(speed * constantSpeedModifier) * currentSpeedModeModifier);
        topRight.setPower((speed * constantSpeedModifier) * currentSpeedModeModifier);
    }

    /**
     * Drives and turns at the same time.
     *
     * @param input A vector that determines linear/rotational speed and direction. First component is linear speed second is rotational speed (counterclockwise +)
     */
    public void turnAndMove(Vector2D input) {
        botLeft.setPower(((input.getX() - input.getY()) * constantSpeedModifier) * currentSpeedModeModifier);
        botRight.setPower(((input.getX() + input.getY()) * constantSpeedModifier) * currentSpeedModeModifier);
        topLeft.setPower(((input.getX() - input.getY()) * constantSpeedModifier) * currentSpeedModeModifier);
        topRight.setPower(((input.getX() + input.getY()) * constantSpeedModifier) * currentSpeedModeModifier);
    }

    /**
     * Stops all movement.
     */
    public void stopMovement(){
        botLeft.setPower(0);
        botRight.setPower(0);
        topLeft.setPower(0);
        topRight.setPower(0);
    }

    /**
     * Sets the power of the left motors.
     *
     * @param speed Power to set the motor to(-1)-(1).
     */
    public void setPowerLeft(double speed){
        botLeft.setPower((speed * constantSpeedModifier) * currentSpeedModeModifier);
        topLeft.setPower((speed * constantSpeedModifier) * currentSpeedModeModifier);
    }

    /**
     * Sets the power of the right motors.
     *
     * @param speed Power to set the motor to(-1)-(1).
     */
    public void setPowerRight(double speed){
        botRight.setPower((speed * constantSpeedModifier) * currentSpeedModeModifier);
        topRight.setPower((speed * constantSpeedModifier) * currentSpeedModeModifier);
    }

    /**
     * Moves forward or backwards for a set time.
     *
     * @param timeMs time to drive for in milliseconds.
     * @param power power to drive at positive power for forward, negative for backwards.
     *
     * @throws DumpsterFireException Throws this exception if time is negative.
     */
    public void driveTime(long timeMs, double power) {
        if(timeMs < 0) {
            throw new DumpsterFireException("HAL is cool, but can't travel back in time. Time must be positive.");
        }

        drive(power);
        waitTime(timeMs);
        stopMovement();
    }

    /**
     * Turns for a set time.
     *
     * @param timeMs time to turn for in milliseconds.
     * @param power power to turn at. positive for counterClockwise and negative for clockwise.
     *
     * @throws DumpsterFireException Throws this exception if time is negative.
     */
    public void turnTime(long timeMs, double power) {
        if(timeMs < 0) {
            throw new DumpsterFireException("HAL is cool, but can't travel back in time. Time must be positive.");
        }

        double startTime = System.currentTimeMillis();
        turn(power);
        waitTime(timeMs);
        stopMovement();
    }

    /**
     * Drives and turns for a set time.
     *
     * @param timeMs Time to turn and drive for in milliseconds.
     * @param input Vector2D that determines direction and rotational speed. (x component is linear speed y is rotational speed)
     *
     * @throws DumpsterFireException Throws this exception if time is negative.
     */
    public void turnAndMoveTime(long timeMs, Vector2D input) {
        if(timeMs < 0) {
            throw new DumpsterFireException("HAL is cool, but can't travel back in time. Time must be positive.");
        }

        turnAndMove(input);
        waitTime(timeMs);
        stopMovement();
    }

    /**
     * Dives using encoders.
     *
     * @param encoderDistance Encoder distance to travel.
     * @param power Double from (-1)-(1) of intensity of the movement positive for forward and negative for reverse.
     *
     * @throws InvalidMoveCommandException Throws this error if the encoders to travel is nonzero and the power is 0.
     * @throws DumpsterFireException Throws this exception if the encoder count is negative.
     */
    public void driveEncoders(final int encoderDistance, double power) {
        if(power == 0 && encoderDistance != 0) {
            throw new InvalidMoveCommandException("Power cannot be zero with a non zero target");
        }

        if (encoderDistance < 0) {
            throw new DumpsterFireException("Where you're going, you don't need roads! (distance must be positive)");
        }

        final int startEncoderPos = topLeft.getCurrentPosition();
        drive(power);
        waitWhile(new Supplier<Boolean>() {
            @Override
            public Boolean get() {
                return Math.abs(topLeft.getCurrentPosition() - startEncoderPos) <= encoderDistance;
            }
        });
        stopMovement();
    }

    /**
     * Turns using encoders.
     *
     * @param encoderDistance Encoder distance to travel.
     * @param power Double from (-1)-(1) of intensity of turn in the turn move(positive for counterclockwise negative for clockwise).
     *
     * @throws DumpsterFireException Throws this exception if the encoder count is negative.
     */
    public void turnEncoders(final int encoderDistance, double power) {

        if(encoderDistance < 0) {
            throw new DumpsterFireException("Where you're going, you don't need roads! (distance must be positive)");
        }

        final int leftStartEncoderPos = topLeft.getCurrentPosition();
        final int rightStartEncoderPos = topRight.getCurrentPosition();
        if(power > 0) {
            turn(power);
            waitWhile(new Supplier<Boolean>() {
                @Override
                public Boolean get() {
                    return Math.abs(topLeft.getCurrentPosition() - leftStartEncoderPos) <= encoderDistance;
                }
            });
            stopMovement();
        }
        if (power < 0){
            turn(power);
            waitWhile(new Supplier<Boolean>() {
                @Override
                public Boolean get() {
                    return Math.abs(topRight.getCurrentPosition() - rightStartEncoderPos) <= encoderDistance;
                }
            });
            stopMovement();
        }
    }

    /**
     * Turn while driving using encoders.
     *
     * @param encoderDistance Encoder distance to travel.
     * @param input Vector2D that determines direction and rotational speed. (x component is linear speed y is rotational speed)
     *
     * @throws DumpsterFireException Throws this exception if the encoder count is negative.
     */
    public void turnAndMoveEncoders(final int encoderDistance, Vector2D input) {

        if(encoderDistance < 0) {
            throw new DumpsterFireException("Where you're going, you don't need roads! (distance must be positive)");
        }

        final int leftStartEncoderPos = topLeft.getCurrentPosition();
        final int rightStartEncoderPos = topRight.getCurrentPosition();
        turnAndMove(input);
        waitWhile(new Supplier<Boolean>() {
            @Override
            public Boolean get() {
                return Math.abs((topLeft.getCurrentPosition() - leftStartEncoderPos)/2 + (topRight.getCurrentPosition() - rightStartEncoderPos)/2) <= encoderDistance;
            }
        });
        stopMovement();
    }


    /**TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO
     * WIP
     */
    @Deprecated
    public void PIDDrive(double kp, double ki, double kd){
        PIDController pid = new PIDController(kp, ki, kd);
        //turnClockwise(pid.getCorrection());
    }

    @Deprecated
    public void PIDTurn(double kp, double ki, double kd, double targetAngle) {
        PIDController pid = new PIDController(kp, ki, kd);
        pid.setSetpoint(targetAngle);
    }

    /**
     * Configures the motors.
     *
     * @param topLeftConfigurationName Configuration name for topLeftMotor
     * @param topRightConfigurationName Configuration name for topRightMotor
     * @param botLeftConfigurationName Configuration name for botLeftMotor
     * @param botRightConfigurationName Configuration name for botRightMotor
     */
    public void setMotorConfiguration(String topLeftConfigurationName, String topRightConfigurationName, String botLeftConfigurationName, String botRightConfigurationName){
        topLeft = robot.hardwareMap.dcMotor.get(topLeftConfigurationName);
        topRight = robot.hardwareMap.dcMotor.get(topRightConfigurationName);
        botLeft = robot.hardwareMap.dcMotor.get(botLeftConfigurationName);
        botRight = robot.hardwareMap.dcMotor.get(botRightConfigurationName);
    }

    /**
     * Sets speedModeModifier.
     *
     * @param speedModeModifier Value to set speedModeModifier to.
     */
    public void setSpeedModeModifier(double speedModeModifier){ this.speedModeModifier = speedModeModifier; }

    /**
     * Sets constantSpeedModifier.
     *
     * @param constantSpeedModifier Value to set constantSpeedModeModifier to.
     */
    public void setConstantSpeedModifier(double constantSpeedModifier){ this.constantSpeedModifier = constantSpeedModifier; }

    /**
     * Sets wether or not to be able to turn and move at the same time.
     *
     * @param turnAndMove Should the robot turn and move at the same time true or false.
     */
    public void setTurnAndMove(boolean turnAndMove) { this.turnAndMove = turnAndMove; }

    /**
     * Sets the double input responsible for moving forward and backwards.
     *
     * @param button Double input responsible for moving forward and backwards.
     *
     * @throws NotDoubleInputException Throws an exception if button does not return double values.
     */
    public void setDriveStick(Button button){
        if(button.isDouble()) {
            inputs.addButton(DRIVESTICK, button);
        }
        else {
            throw new NotDoubleInputException("driveStick was not set to a double input");
        }
    }

    /**
     * Sets the double input responsible for turning right and left.
     *
     * @param button Double input responsible for turning right and left.
     *
     * @throws NotDoubleInputException Throws an exception if button does not return double values.
     */
    public void setTurnStick(Button button){
        if(button.isDouble()) {
            inputs.addButton(TURNSTICK, button);
        }
        else {
            throw new NotDoubleInputException("turnStick was not set to a double input");
        }
    }

    /**
     * Sets the boolean input responsible for toggling speedMode.
     *
     * @param button The boolean input responsible for toggling speedMode.
     *
     * @throws NotBooleanInputException Throws an exception if button does not return boolean values.
     */
    public void setSpeedMode(Button button){
        if(button.isBoolean()) {
            inputs.addButton(SPEEDMODEBUTTON, button);
        }
        else {
            throw new NotBooleanInputException("speedModeButton was not set to a boolean input");
        }
    }

    /**
     * Pulls teleop config settings.
     */
    private void setUsingConfigs() {
        inputs = robot.pullControls(this);
        ConfigData data = robot.pullNonGamepad(this);

        setTurnAndMove(data.getData("Turn and Move",Boolean.class));
        if (!useSpecific) {
            setConstantSpeedModifier(data.getData("ConstantSpeedModifier",Double.class));
            setSpeedModeModifier(data.getData("SpeedModeModifier",Double.class));
        }
    }

    /**
     * Pulls autonomous config settings.
     */
    private void setUsingConfigsAutonomous() {
        ConfigData data = robot.pullNonGamepad(this);

        setConstantSpeedModifier(data.getData("ConstantSpeedModifier",Double.class));
    }

    /**
     * Gets the top left motor's encoder position.
     *
     * @return The top left motor's encoder position.
     */
    public int getTopLeftMotorEncoderPos(){
        return topLeft.getCurrentPosition();
    }

    /**
     * Gets the top right motor's encoder position.
     *
     * @return The top right motor's encoder position.
     */
    public int getTopRightMotorEncoderPos(){
        return topRight.getCurrentPosition();
    }

    /**
     * Gets the bottom left motor's encoder position.
     *
     * @return The bottom left motor's encoder position.
     */
    public int getBotLeftMotorEncoderPos(){
        return botLeft.getCurrentPosition();
    }

    /**
     * Gets the top right motor's encoder position.
     *
     * @return The top right motor's encoder position.
     */
    public int getBotRightMotorEncoderPos(){
        return botRight.getCurrentPosition();
    }

    /**
     * Gets an array containing all of the motor encoder positions.
     *
     * @return An array containing all of the motor encoder positions.
     */
    public int[] getMotorEncoderPoses(){
        return new int[]{topLeft.getCurrentPosition(), topRight.getCurrentPosition(), botLeft.getCurrentPosition(), botRight.getCurrentPosition()};
    }

    /**
     * Gets an array containing all of the motors.
     *
     * @return An array containing all of the motors. (topLeft, topRight, botLeft, botRight)
     */
    public DcMotor[] getMotors(){
        return new DcMotor[]{topLeft, topRight, botLeft, botRight};
    }

    /**
     * The teleop configuration settings.
     *
     * @return The teleop configuration settings.
     */
    @TeleopConfig
    public static ConfigParam[] teleOpConfig() {

        if(useSpecific) {
            return new ConfigParam[]{
                    new ConfigParam(DRIVESTICK, Button.DoubleInputs.left_stick_y),
                    new ConfigParam(TURNSTICK, Button.DoubleInputs.right_stick_x),
                    new ConfigParam(SPEEDMODEBUTTON, Button.BooleanInputs.noButton),
                    new ConfigParam("Turn and Move", ConfigParam.BOOLEAN_MAP, true)
            };
        }
        else {
            return new ConfigParam[]{
                    new ConfigParam(DRIVESTICK, Button.DoubleInputs.left_stick_y),
                    new ConfigParam(TURNSTICK, Button.DoubleInputs.right_stick_x),
                    new ConfigParam(SPEEDMODEBUTTON, Button.BooleanInputs.noButton),
                    new ConfigParam("Turn and Move", ConfigParam.BOOLEAN_MAP, true),
                    new ConfigParam("SpeedModeModifier", ConfigParam.numberMap(0, 100, .05), 1.0),
                    new ConfigParam("ConstantSpeedModifier", ConfigParam.numberMap(0, 100, .05), 1.0)
            };
        }
    }

    /**
     * The autonomous configuration settings.
     *
     * @return The autonomous configuration settings.
     */
    @AutonomousConfig
    public static ConfigParam[] autonomousConfig() {
        return new ConfigParam[]{
                new ConfigParam("ConstantSpeedModifier", ConfigParam.numberMap(0,100, .05), 1.0)
        };
    }

    /**
     * A parameters class that is used to pass options into the drive system.
     */
    public static final class Params implements BaseParam {

        //Motor config names to be used in TankDrive to set the motors
        private String botLeftMotor, botRightMotor, topLeftMotor, topRightMotor;
        //Array of buttons to set the buttons to for the TankDrive class [1] is driveStick, [2] is turnStick, and [3] is speedModeButton.
        private Button[] buttonsToSet = new Button[3];
        //A boolean value specifying whether the robot is allowed to turn and move at the same time.
        private boolean turnAndMove;
        //Various double values for speed control.
        private double speedModeModifier = 1, constantSpeedModifier = 1;

        /**
         * Constructor for Params.
         *
         * @param topLeftMotorConfiguration The top left motor's configuration name.
         * @param topRightMotorConfiguration The top right motor's configuration name.
         * @param botLeftMotorConfiguration The bottom left motor's configuration name.
         * @param botRightMotorConfiguration The bottom right motor's configuration name.
         */
        public Params(String topLeftMotorConfiguration, String topRightMotorConfiguration, String botLeftMotorConfiguration, String botRightMotorConfiguration) {
            topLeftMotor = topLeftMotorConfiguration;
            topRightMotor = topRightMotorConfiguration;
            botLeftMotor = botLeftMotorConfiguration;
            botRightMotor = botRightMotorConfiguration;
            setDefaultButtons();
        }

        /**
         * Set whether the drive can turn and move at the same time.
         *
         * @param turnAndMove Whether the drive can turn and move at the same time.
         * @return This instance of Params.
         */
        public Params setTurnAndMove(boolean turnAndMove) {
            this.turnAndMove = turnAndMove;
            return this;
        }

        /**
         * Set the drive's speed mode modifier.
         *
         * @param speedModeModifier The drive's speed mode modifier.
         * @return This instance of Params.
         */
        public Params setSpeedModeModifier(double speedModeModifier) {
            this.speedModeModifier = speedModeModifier;
            return this;
        }

        /**
         * set the drive's constant speed modifier.
         *
         * @param constantSpeedModifier The drive's constant speed modifier.
         * @return This instance of Params.
         */
        public Params setConstantSpeedModifier(double constantSpeedModifier) {
            this.constantSpeedModifier = constantSpeedModifier;
            return this;
        }

        /**
         * Set the drive's drivestick button.
         *
         * @param driveStick The drive button.
         * @return This instance of Params.
         *
         * @throws NotDoubleInputException Throws this exception when the provided button does not return double values.
         */
        public Params setDriveStick(Button driveStick) {
            if(!driveStick.isDouble()) {
                throw new NotDoubleInputException("DriveStick must be a double input.");
            }
            buttonsToSet[0] = driveStick;
            ArrayList<String> s = new ArrayList<>();
            return this;
        }

        /**
         * Set the drive's turnstick button.
         *
         * @param turnStick The turnstick button.
         * @return This instance of Params.
         *
         * @throws NotDoubleInputException Throws this exception when the provided button does not return double values.
         */
        public Params setTurnStick(Button turnStick) {
            if(!turnStick.isDouble()) {
                throw new NotDoubleInputException("TurnStick must be a double input.");
            }
            buttonsToSet[1] = turnStick;
            return this;
        }

        /**
         * Set the drive's speed mode button.
         *
         * @param speedModeButton The speed mode button.
         * @return This instance of Params.
         *
         * @throws NotBooleanInputException Throws this exception when the provided button does not return boolean values.
         */
        public Params setSpeedModeButton(Button speedModeButton) {
            if(!speedModeButton.isBoolean()) {
                throw new NotBooleanInputException("SpeedModeButton must be a boolean input.");
            }
            buttonsToSet[2] = speedModeButton;
            return this;
        }

        /**
         * Set all buttons to default values.
         */
        private void setDefaultButtons(){
            buttonsToSet[0] = new Button(1, Button.DoubleInputs.left_stick_y);
            buttonsToSet[1] = new Button(1, Button.DoubleInputs.right_stick_x);
            buttonsToSet[2] = new Button(1, Button.BooleanInputs.noButton);
        }
    }

    /**
     * A parameters class that is used to pass options into the drive system when config is being used but you want specific values for certain numbers.
     */
    public static final class NumberParams implements BaseParam {

        //Motor config names to be used in QuadWheelDrive to set the motors
        private String topLeftMotor, topRightMotor, botLeftMotor, botRightMotor;
        //Various double values for speed control.
        private double speedModeModifier = 1, constantSpeedModifier = 1;

        /**
         * Constructor for NumberParams.
         *
         * @param topLeftMotorConfiguration The top left motor's configuration name.
         * @param topRightMotorConfiguration The top right motor's configuration name.
         * @param botLeftMotorConfiguration The bottom left motor's configuration name.
         * @param botRightMotorConfiguration The bottom right motor's configuration name.
         */
        public NumberParams(String topLeftMotorConfiguration, String topRightMotorConfiguration, String botLeftMotorConfiguration, String botRightMotorConfiguration) {
            topLeftMotor = topLeftMotorConfiguration;
            topRightMotor = topRightMotorConfiguration;
            botLeftMotor = botLeftMotorConfiguration;
            botRightMotor = botRightMotorConfiguration;

            useSpecific = true;
        }

        /**
         * Set the drive's speed mode modifier.
         *
         * @param speedModeModifier The drive's speed mode modifier.
         * @return This instance of Params.
         */
        public NumberParams setSpeedModeModifier(double speedModeModifier) {
            this.speedModeModifier = speedModeModifier;
            return this;
        }

        /**
         * set the drive's constant speed modifier.
         *
         * @param constantSpeedModifier The drive's constant speed modifier.
         * @return This instance of Params.
         */
        public NumberParams setConstantSpeedModifier(double constantSpeedModifier) {
            this.constantSpeedModifier = constantSpeedModifier;
            return this;
        }
    }
}