package com.qualcomm.robotcore.eventloop.opmode;

import java.lang.annotation.*;

/**
 * Provides an easy and non-centralized way of contributing to the OpMode list
 * shown on an FTC Driver Station. While {@link Autonomous} and {@link TeleOp} annotations
 * can be placed on your *own* classes to register them, to register classes found
 * in libraries *other* than your own it is best to use a mechanism that does not require
 * that you modify source code in that other library. OpModeRegistrar provides such a
 * mechanism.
 * <p>Place an OpModeRegistrar annotation on a static method in your code that accepts a
 * parameter of type {@link OpModeManager} or {@link }, and that
 * method will be automatically called at the right time to register OpModes. You can
 * use any of the register() methods that exist on the {@link }
 * to register OpModes.</p>
 */
@Documented
@Target(ElementType.METHOD)
@Retention(RetentionPolicy.RUNTIME)
public @interface OpModeRegistrar
    {
    }
