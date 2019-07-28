package com.qualcomm.robotcore.eventloop.opmode;

import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

/**
 * Op Mode classes must be annotated with either Autonomous or TeleOp in order to be displayed in the dropdown box
 * of available op modes.
 */
@Target(ElementType.TYPE)
@Retention(RetentionPolicy.RUNTIME)
public @interface TeleOp {
    String name();
    String group() default "default";
}
