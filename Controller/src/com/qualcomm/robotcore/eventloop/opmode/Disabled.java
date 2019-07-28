package com.qualcomm.robotcore.eventloop.opmode;

import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

/**
 * Annotate an op mode class with Disabled to prevent it from being displayed in the drop-down box of available
 * op modes.
 */
@Target(ElementType.TYPE)
@Retention(RetentionPolicy.RUNTIME)
public @interface Disabled {
}
