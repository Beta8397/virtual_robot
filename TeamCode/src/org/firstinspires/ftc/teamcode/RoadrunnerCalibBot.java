package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import system.gui.menus.examplemenu.ExampleMenu;
import system.robot.Robot;
import system.robot.localizer.*;
import system.robot.roadrunner_util.AxesSigns;
import system.robot.roadrunner_util.CoordinateMode;
import system.robot.roadrunner_util.Encoder;
import system.robot.roadrunner_util.RoadrunnerConfig;
import system.robot.subsystems.drivetrain.DriveConfig;
import system.robot.subsystems.drivetrain.MecanumDrive;
import system.robot.subsystems.drivetrain.MecanumDriveSimple;
import util.control.Button;
import util.math.units.HALAccelerationUnit;

import static java.lang.Math.PI;

public class RoadrunnerCalibBot extends Robot {
    public MecanumDrive drive;
    public RoadrunnerCalibBot(OpMode opMode) {
        super(opMode);

        drive = new MecanumDrive(this,
                new RoadrunnerConfig(2,1,15,1120,133.9),
                "front_left_motor",
                "front_right_motor",
                "back_left_motor",
                "back_right_motor", true);
        drive.setLocalizer(new HolonomicDriveEncoderIMULocalizer(
                this,
                drive,
                "imu",
                "front_left_motor",
                "front_right_motor",
                "back_left_motor",
                "back_right_motor"
        ));
    }
}
