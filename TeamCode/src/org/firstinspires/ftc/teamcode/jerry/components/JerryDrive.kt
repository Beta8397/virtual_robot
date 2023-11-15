package org.firstinspires.ftc.teamcode.jerry.components

import com.qualcomm.robotcore.hardware.DcMotorEx
import org.firstinspires.ftc.teamcode.common.BunyipsOpMode
import org.firstinspires.ftc.teamcode.common.OldMecanumDrive

/**
 * Jerry robot drivetrain operation module.
 */
class JerryDrive(
    opMode: BunyipsOpMode,
    bl: DcMotorEx, br: DcMotorEx,
    fl: DcMotorEx, fr: DcMotorEx
) : OldMecanumDrive(opMode, fl, bl, fr, br)