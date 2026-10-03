package org.firstinspires.ftc.teamcode.pedropathing.tuning;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {

    /*
     * THESE ARE ONLY FOR USE IN RUNNING THE TUNING OPMODES IN VIRTUAL_ROBOT, WHICH
     * ARE DISABLED BY DEFAULT. THERE SHOULD BE NO NEED TO RETUNE.  THEY ARE
     * NOT NEEDED FOR A REAL ROBOT!! THEY ALSO ARE NOT NEEDED FOR YOUR CUSTOM OPMODES
     * OR THE TESTING OPMODES IN VIRTUAL_ROBOT.
     */
    public static double tuningHeadingLinear = 0.0;
    public static double tuningHeadingQuadratic = 0.0;
    public static double tuningHeading = 0.0;
    public static double tuningForwardLinear = 0.0;
    public static double tuningForwardQuadratic = 0.0;
    public static double tuningStrafeLinear = 0.0;
    public static double tuningStrafeQuadratic = 0.0;

    /*
     * THE TUNING VALUES BELOW ARE FOR THE MecDynamicBot CONFIGURATION OF
     * VIRTUAL_ROBOT. THAT IS THE RECOMMENDED CONFIGURATION FOR USING
     * PEDROPATHING.
     *
     * THERE SHOULD BE NO NEED TO RETUNE, AS THESE HAVE BEEN CREATED BY RUNNING
     * THE TUNING OPMODES OF THE PEDRO QUICKSTART (ADAPTED AS NEEDED FOR THE
     * VIRTUAL_ROBOT PROJECT).
     */

    public static MecanumConfig drivetrainConfig = new MecanumConfig(
            c -> {
                c.frontLeftName.set("front_left_motor");
                c.backLeftName.set("back_left_motor");
                c.frontRightName.set("front_right_motor");
                c.backRightName.set("back_right_motor");
                c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
                c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
                c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
                c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
                c.manualBrakeMode.set(true);
            }
    );
    public static PinpointConfig localizerConfig = new PinpointConfig(
            c -> {
                c.name.set("pinpoint");
                c.xPodOffset.set(3.937);
                c.yPodOffset.set(3.937);
                c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
                c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
            }
    );
    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.29);
                Controller secondaryTranslationalForward = Controller.proportional(0.11);
                Controller primaryTranslationalLateral = Controller.proportional(0.29);
                Controller secondaryTranslationalLateral = Controller.proportional(0.11);
//                c.forwardTranslational.set(secondaryTranslationalForward);
//                c.strafeTranslational.set(secondaryTranslationalLateral);
                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));
                c.coast.set(Controller.proportionalFeedforward(0.02));
                c.brake.set(Controller.proportionalFeedforward(0.017));
                c.headingFeedback.set(Controller.proportional(3.14));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.045, 0.0022));
                c.linearBrakeCoefficients.set(Matrix.diag(0.11, 0.14));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0017, 1.6e-4));
                c.maxAchievableForwardVelocity.set(49.9);
                c.maxAchievableStrafeVelocity.set(49.9);
                c.naturalForwardDeceleration.set(24.4);
                c.naturalStrafeDeceleration.set(24.4);
            }
    );

    public static Follower create(HardwareMap h) {
        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }
}
