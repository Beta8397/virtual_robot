package system.config;

import system.robot.Robot;

import java.lang.annotation.Documented;
import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

/**
 * A method annotation used to denote a method that specifies the autonomous config options for a subsystem.
 * <p>
 * Creation Date: 8/18/19
 *
 * @author Cole Savage, Level Up
 * @version 1.0.0
 * @see TeleopConfig
 * @see HALConfig
 * @see Robot
 * @see system.robot.SubSystem
 * @since 1.0.0
 */
@Documented
@Retention(RetentionPolicy.RUNTIME)
@Target(ElementType.METHOD)
public @interface AutonomousConfig {}
