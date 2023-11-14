package org.firstinspires.ftc.teamcode.jerry.components

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.Servo
//import com.qualcomm.robotcore.hardware.TouchSensor
import org.firstinspires.ftc.teamcode.common.BunyipsComponent
import org.firstinspires.ftc.teamcode.common.BunyipsOpMode
import org.firstinspires.ftc.teamcode.common.While

/**
 * Improved and refactored arm control system for Jerry robot, using relative capture instead of index-based movement
 * @author Lucas Bubner, 2023
 */
class JerryLift(
    opMode: BunyipsOpMode,
    private var mode: ControlMode,
    private var claw: Servo,
    private var arm1: DcMotorEx,
    private var arm2: DcMotorEx,
//    private var limit: TouchSensor
) : BunyipsComponent(opMode) {
    var power: Double = POSITIONAL_POWER
        set(power) {
            // Paranoia, ensure the arm doesn't go past the hard limit when taking power commands
            // from various and untrusted sources
            if (power > 0 && (arm1.currentPosition + arm2.currentPosition) / 2 >= HARD_LIMIT)
                return
            // Gravity will help us out, do not allow negative power to exceed -0.2
            field = power.coerceIn(-0.2, 1.0)
        }
    private val motors = arrayOf(arm1, arm2)
    private var targetPosition: Double = 0.0
    private var holdPosition: Int? = null

    private var deltaTimeout = 0
    private val resetLock = While(
        {
            // Check if the delta is above 0 for 30 consecutive loops, if so, we have hit the limit
            val prev = (motors[0].currentPosition + motors[1].currentPosition) / 2
            val delta = (motors[0].currentPosition + motors[1].currentPosition) / 2 - prev
            if (delta >= 0) {
                deltaTimeout++
            } else {
                deltaTimeout = 0
            }
            // Reset lock when limit is pressed, delta is stagnant, or when manually interrupted
            !opMode.gamepad2.right_bumper && deltaTimeout < 30
        },
        {
            opMode.addTelemetry(
                String.format(
                    "LIFT IS RESETTING... PRESS GAMEPAD2.RIGHT_BUMPER TO CANCEL! ENCODER VALUES: %d, %d",
                    arm1.currentPosition,
                    arm2.currentPosition
                )
            )
        },
        {
            for (motor in motors) {
                // Finally, we reset the motors and we are now zeroed out again.
                motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
                motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
                motor.targetPosition = 0
            }

            targetPosition = 0.0
            deltaTimeout = 0
        },
        5.0
    )

    private val releaseLock = While(
        {
            // Release lock when motors have reached the target or manually interrupted
            arm1.isBusy && arm2.isBusy && !opMode.gamepad2.right_bumper
        },
        {
            opMode.addTelemetry(
                String.format(
                    "LIFT IS RECALLING TO HOLD POSITION %d... PRESS GAMEPAD2.RIGHT_BUMPER TO CANCEL! ENCODER VALUES: %d, %d",
                    holdPosition,
                    arm1.currentPosition,
                    arm2.currentPosition
                )
            )
        },
        {
            for (motor in motors) {
                motor.power = 0.0
                motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
            }

            opMode.log("lift released to $holdPosition")
            holdPosition = null
        },
        5.0
    )

    // Handle both manual and positional control modes
    enum class ControlMode {
        MANUAL, POSITIONAL
    }

    init {
        for (motor in motors) {
            // Set initial parameters
            motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
            motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
            if (mode == ControlMode.POSITIONAL) {
                motor.power = power
            } else {
                power = 0.0
                motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
            }
        }
        // Run an initial calibration to zero out
        reset()
    }

    /**
     * Reset the arm back to a zeroed position with bounds detection/limit switch cutoff.
     * This will ensure that a saved position is still accurate even after encoder drift.
     */
    fun reset() {
        close()
        for (motor in motors) {
            motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
            // Allow gravity and light motor power to pull the arm down
            motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT
            motor.power = -0.1
        }

        resetLock.engage()
    }

    /**
     * Check if the lift is currently trying to move to a position
     */
    fun isBusy(): Boolean {
        if (mode == ControlMode.MANUAL) {
            return power != 0.0 || resetLock.running() || releaseLock.running()
        }

        return motors[0].isBusy ||
                motors[1].isBusy ||
                resetLock.running() ||
                targetPosition != motors[0].targetPosition.toDouble() ||
                targetPosition != motors[1].targetPosition.toDouble()
    }

    /**
     * Call to open claw system and allow picking up of items (allow servos to swing to open)
     */
    fun open() {
        claw.position = 0.0
        opMode.addTelemetry("Claws are opening...")
    }

    /**
     * Call to close claw system and pick up an object by setting the target position to the closed
     * position. Using Servo mode as opposed to CRServo mode.
     */
    fun close() {
        claw.position = 1.0
        opMode.addTelemetry("Claws are closing...")
    }

    /**
     * Primary function to move the arm during TeleOp, takes in a controller input delta and adjusts the arm position
     */
    fun delta(dy: Double) {
        if (mode != ControlMode.MANUAL) {
            throw IllegalStateException("JerryLift: delta() method can only be used in manual mode")
        }
        if (dy < 0) {
            // Arm is ascending, auto close claw
            // Auto-close removed 24/10/23 as it is now invasive
//            if (claw.position == 0.0)
//                close()
            // Check if the arm has reached maximum limit
            if ((arm1.currentPosition + arm2.currentPosition) / 2 >= HARD_LIMIT) {
                // If so, stop the arm from moving further upwards
                power = 0.0
                return
            }
        }
        if (dy > 0) {
            // If the arm is descending and the limit switch is pressed, don't change motor speed
            return
        }
        power = -dy / DAMPENER
    }

    /**
     * Autonomous method of adjusting the arm position, takes in a percentage and moves the arm to that position
     * The maximum 100% position is represented by the internal HARD_LIMIT constant
     * @param percent The percentage of the maximum position to move to
     */
    fun set(percent: Int) {
        if (mode != ControlMode.POSITIONAL) {
            throw IllegalStateException("JerryLift: set() method can only be used in positional mode")
        }
        if (percent !in 0..100) {
            throw IllegalArgumentException("JerryLift: set() method must be between 0 and 100%")
        }
        if (percent == 0) {
            reset()
            return
        }
        targetPosition = percent / 100.0 * HARD_LIMIT
    }

    /**
     * Set a capture of the arm position
     */
    fun capture() {
        if (mode != ControlMode.MANUAL) {
            throw IllegalStateException("JerryLift: capture() method can only be used in manual mode")
        }
        val armPos = (arm1.currentPosition + arm2.currentPosition) / 2
        holdPosition = minOf(armPos, HARD_LIMIT)
        opMode.log("lift captured at $holdPosition")
    }

    /**
     * Return the arm to the last captured position
     */
    fun release() {
        if (mode != ControlMode.MANUAL) {
            throw IllegalStateException("JerryLift: release() method can only be used in manual mode")
        }
        if (holdPosition == null) {
            opMode.log("lift released but no capture was found")
            return
        }

        opMode.log("lift releasing to $holdPosition...")

        // Configure motors for positional control
        for (motor in motors) {
            motor.targetPosition = holdPosition!!
            motor.mode = DcMotor.RunMode.RUN_TO_POSITION
            motor.power = POSITIONAL_POWER
        }

        releaseLock.engage()
    }

    fun update() {
        // While a reset or release is in progress, do not allow any other operations
        if (resetLock.run()) return
        if (releaseLock.run()) return

        opMode.addTelemetry(
            String.format(
                "Lift (pos1, pos2, target, capture): %d, %d, %s, %s",
                arm1.currentPosition,
                arm2.currentPosition,
                targetPosition.toInt(),
                holdPosition ?: "N/A"
            )
        )

        when (mode) {
            ControlMode.POSITIONAL -> {
                // Ensure the arm does not over swing
                if (targetPosition > HARD_LIMIT) {
                    targetPosition = HARD_LIMIT.toDouble()
                }

                // Apply changes to the arm system
                for (motor in motors) {
                    motor.targetPosition = targetPosition.toInt()
                    motor.mode = DcMotor.RunMode.RUN_TO_POSITION
                }
            }

            ControlMode.MANUAL -> {
                // Lock motor if there is no input
                if (power == 0.0) {
                    for (motor in motors) {
                        motor.targetPosition =
                            (motors[0].currentPosition + motors[1].currentPosition) / 2
                        motor.mode = DcMotor.RunMode.RUN_TO_POSITION
                        motor.power = POSITIONAL_POWER
                    }
                } else {
                    // Feed power into motor
                    for (motor in motors) {
                        motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
                        motor.power = power
                    }
                }
            }
        }
    }

    companion object {
        // Maximum encoder value of extension
        private const val HARD_LIMIT = 200

        // Delta speed dampener for manual mode
        private const val DAMPENER = 1.5

        // Maximum speed of the arm in autonomous mode
        private const val POSITIONAL_POWER = 0.2
    }
}