package system.config;

import java.lang.annotation.Documented;
import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

/**
 * A class annotation used to denote that an opmode is a stand-alone program.
 * <p>
 * Creation Date: 8/17/19
 *
 * @author Cole Savage, Level Up
 * @version 1.0.0
 * @see system.robot.HALProgram
 * @see system.robot.Robot
 * @see system.gui.menus.configmenu.ConfigStartingMenu
 * @since 1.0.0
 */
@Documented
@Retention(RetentionPolicy.RUNTIME)
@Target(ElementType.TYPE)
public @interface StandAlone {}
