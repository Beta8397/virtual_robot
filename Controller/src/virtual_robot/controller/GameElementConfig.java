package virtual_robot.controller;

import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

@Target(ElementType.TYPE)
@Retention(RetentionPolicy.RUNTIME)
public @interface GameElementConfig {
    String name();
    String filename();
    Class<? extends Game> forGame();
    int numInstances() default 1;
}
