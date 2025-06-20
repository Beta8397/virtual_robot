package pedroPathing.constants;

import com.pedropathing.localization.*;
import com.pedropathing.localization.constants.*;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class LConstants {
    static {
        PinpointConstants.forwardY = 1;
        PinpointConstants.strafeX = -2.5;
        PinpointConstants.distanceUnit = DistanceUnit.INCH;
        PinpointConstants.hardwareMapName = "pinpoint";
        PinpointConstants.useYawScalar = false;
        PinpointConstants.yawScalar = 1.0;
        PinpointConstants.useCustomEncoderResolution = false;
        PinpointConstants.encoderResolution = GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;
        PinpointConstants.customEncoderResolution = 13.26291192;
        PinpointConstants.forwardEncoderDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD;
        PinpointConstants.strafeEncoderDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD;


//        ThreeWheelConstants.forwardTicksToInches = .001989436789;
//        ThreeWheelConstants.strafeTicksToInches = .001989436789;
//        ThreeWheelConstants.turnTicksToInches = .001989436789;
//        ThreeWheelConstants.leftY = 1;
//        ThreeWheelConstants.rightY = -1;
//        ThreeWheelConstants.strafeX = -2.5;
//        ThreeWheelConstants.leftEncoder_HardwareMapName = "leftFront";
//        ThreeWheelConstants.rightEncoder_HardwareMapName = "rightRear";
//        ThreeWheelConstants.strafeEncoder_HardwareMapName = "rightFront";
//        ThreeWheelConstants.leftEncoderDirection = Encoder.REVERSE;
//        ThreeWheelConstants.rightEncoderDirection = Encoder.REVERSE;
//        ThreeWheelConstants.strafeEncoderDirection = Encoder.FORWARD;
    }
}




