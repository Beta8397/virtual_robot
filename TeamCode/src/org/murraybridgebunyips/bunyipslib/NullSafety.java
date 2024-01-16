package org.murraybridgebunyips.bunyipslib;

import static org.murraybridgebunyips.bunyipslib.Text.formatString;

import org.murraybridgebunyips.bunyipslib.roadrunner.util.Encoder;

import java.util.ArrayList;
import java.util.List;

/**
 * Null safety utilities for robot components.
 *
 * @author Lucas Bubner, 2023
 * @see ErrorUtil
 */
public class NullSafety {
    public static final List<String> unusableComponents = new ArrayList<>();

    /**
     * Ensure that all objects are not null.
     *
     * @param objs Objects to check for null
     * @return Whether all objects are not null
     */
    public static boolean assertNotNull(Object... objs) {
        for (Object o : objs) {
            if (o == null) {
                Dbg.warn("Assertion by NullSafety.assertNotNull() failed.");
                return false;
            }
        }
        return true;
    }

    public static boolean assertNotNull(List<Object> objs) {
        return assertNotNull(objs.toArray());
    }

    /**
     * Ensure a component is safe to instantiate by checking for null objects.
     * Errors caused by null objects are logged and the component is added to the unusable components list.
     * Components in the unusable components list will not have their errors logged.
     *
     * @param opMode BunyipsOpMode overhead instance
     * @param T      Class of the component (e.g. Cannon.class)
     * @param objs   Objects to check for null
     * @return Whether the component is safe to instantiate
     */
    public static boolean assertComponentArgs(BunyipsOpMode opMode, Class<?> T, Object... objs) {
        for (Object o : objs) {
            if (o == null) {
                return reportUnusable(opMode, T);
            } else if (o instanceof Encoder) {
                if (((Encoder) o).isNull()) {
                    return reportUnusable(opMode, T);
                }
            }
        }
        return true;
    }

    private static boolean reportUnusable(BunyipsOpMode opMode, Class<?> component) {
        opMode.addRetainedTelemetry(formatString("! COM_FAULT: % failed to instantiate due to null constructor arguments", component.getSimpleName()));
        opMode.log("error: % is null. attempting to suppress errors...", component.getSimpleName());
        Dbg.error(formatString("% is null, adding to unusable components...", component.getSimpleName()));
        if (!unusableComponents.contains(component.getSimpleName()))
            unusableComponents.add(component.getSimpleName());
        return false;
    }
}
