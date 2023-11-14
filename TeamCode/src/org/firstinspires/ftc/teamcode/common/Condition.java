package org.firstinspires.ftc.teamcode.common;

/**
 * Functional interface for a condition used in While polling
 */
@FunctionalInterface
public interface Condition {
    boolean check();
}
